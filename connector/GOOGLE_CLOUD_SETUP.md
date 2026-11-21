# Google Cloud integration guide for Dashcam Connect

Use this guide to take one paired Jetson from a working Wi-Fi connection to a telemetry session on a phone using mobile data. Start with **Plan A (On demand)**. The powered Jetson listens for requests in Firestore. When an invited viewer connects, the Jetson opens an outbound connection to the Cloud Run relay. It sends camera frames only if a viewer requests the camera. The planned browser dashboard uses HTTPS and the Android app uses its own cloud connection. Neither client needs a VPN installation or router port forwarding.

## Where this pilot stands — 21 November 2026

| Item | Evidence and next action |
|---|---|
| Google Cloud | The pilot project is `jetson-dashcam-handler`. Cloud Shell confirmed the `(default)` Firestore Native, Standard database in Singapore (`asia-southeast1`) and all five APIs listed in step 5. Reuse this project. Steps 1–5 are also a reference for a fresh installation. |
| Firebase and billing | Web and Android apps are registered. The console showed Google sign-in enabled, and the owner completed a monthly billing alert. Confirm billing linkage in step 2 before the first deployment. |
| Android sign-in | The local configuration matches the pilot project and `com.dashcam.connector`. The copy inspected on this Jetson lacks an Android OAuth client entry for the package. Check the signing certificate and refresh the file on the APK build machine in step 6. |
| Jetson and phone | The LAN connector answered locally. The current phone test is in progress. Record its result in step 7. The Python and native C++ Plan A lifecycle tests passed locally. |
| Remote viewing | Device enrollment, the Firestore listener, authenticated relay, dashboard sign-in and cloud transports still need implementation. A remote URL and a cloud-enabled APK are deliverables of step 8. |

## Prerequisites on your side

| When | What to have ready | How to check |
|---|---|---|
| Now | Access to the existing project in Firebase and Google Cloud, its billing alert recipient and Cloud Shell. | Select the pilot project in both consoles. Steps 2 and 5 give read-only checks for billing and the database. |
| Before building the Android pilot | The computer and signing keystore used for the test APK, Android Studio or an equivalent command-line build environment, a compatible Gradle JDK (JDK 17 is the project baseline), and Android SDK Platform 37. Use the repository's Gradle wrapper. | Open the checkout's `android/` project and complete step 6 on that computer. A different machine may use a different debug keystore. |
| Before testing a real Jetson | Shell access to one powered Jetson, its existing local pairing, a working recorder for the live-data test, and a network with outbound DNS and TLS on TCP 443. The device clock must be accurate for certificates and expiring requests. | Finish the Wi-Fi test in step 7. On the Jetson, `timedatectl status` shows clock and synchronization state. |
| Before first deployment | A deployer who can build and deploy the relay and attach its runtime service account, and an administrator who can grant the build and runtime identities their required access. Cloud Shell is sufficient for the administrator. | Identify who manages project IAM. Use the source-deployment permissions for the chosen build identity. Give the relay a dedicated runtime identity with the Firestore access it needs. [Cloud Run source deployment](https://docs.cloud.google.com/run/docs/deploying-source-code) |
| Before the first external viewing session | One Google account to invite to one vehicle, an Android phone with mobile data and Google sign-in, and a browser with internet access for the web test. | Keep the Jetson on its own internet connection. The phone should be able to turn off Wi-Fi and still reach the internet. The second Jetson and remaining viewers can join after the first test passes. |

Keep the APK signing key and its password on the build machine. The LAN pairing link contains a reusable local credential. Record the test result without copying that link into an issue or log. Enrollment will issue a separate device credential. The phone and Jetson never need a cloud administrator key.

## 1. Choose the project

Sign into the [Google Cloud Console](https://console.cloud.google.com/) with the account that should own the pilot. Open the project selector and choose **New Project**. A suggested display name is **Dashcam Connect Pilot**. Google proposes a globally unique project ID; check it before creating the project and keep a copy. If you already have a dedicated project, select it and use that ID throughout.

Open [Cloud Shell](https://shell.cloud.google.com/) in the same account. Click **Authorize** if prompted. In the terminal, enter the actual project ID when asked. These commands use explicit project flags so another shell's default project cannot receive the changes.

```bash
read -rp 'Dashcam project ID: ' DASHCAM_PROJECT_ID
gcloud projects describe "$DASHCAM_PROJECT_ID" \
  --format='table(projectId,name,lifecycleState)'
```

Proceed when the expected ID and `ACTIVE` appear. In a company account, project creation or billing linkage may require your organization administrator.

## 2. Link billing and create a pilot alert

In [Billing](https://console.cloud.google.com/billing), create or select a billing account and link **only the intended pilot project**. Google handles payment details directly. In **Budgets & alerts**, create a monthly budget scoped to the dashcam project. For this pilot, the selected amount was **100,000 VND** with actual-spend thresholds of 50%, 90%, and 100%. If the scope shows **All projects**, it tracks the billing account’s selected projects collectively. Choose an email recipient you check. An alerts-only budget emails you about spend; charges can continue after a threshold. Confirm the link in Cloud Shell. [Budget documentation](https://docs.cloud.google.com/billing/docs/how-to/budgets)

```bash
gcloud billing projects describe "$DASHCAM_PROJECT_ID" --format='value(billingEnabled)'
```

Expected result: `True`. While the relay is under development, there is no need to reserve a VM, external IP, load balancer or minimum Cloud Run instance.

## 3. Add Firebase and prepare sign-in

1. Open the [Firebase console](https://console.firebase.google.com/), choose the create-project flow, then **Add Firebase to Google Cloud project**. Select the exact project ID from step 1. Review the Firebase terms in your browser. Analytics is optional for this pilot. Adding Firebase to an existing project is a permanent project choice. [Firebase setup for existing projects](https://firebase.google.com/docs/projects/use-firebase-with-existing-cloud-project)
2. In **Authentication → Sign-in method**, enable **Google**, choose the support email if prompted, and save. The planned relay will grant access to vehicles separately for each invited user. Signing into Firebase alone should never grant access to a vehicle. [Google sign-in setup](https://firebase.google.com/docs/auth/web/google-signin)
3. In **Project settings → Your apps**, register a web app named **Dashcam Web**. Firebase Hosting is optional; the plan serves the web UI from Cloud Run. Register an Android app with the repository's application ID `com.dashcam.connector`. For Android Google sign-in, add the SHA-1 fingerprint of the certificate used to sign the test APK. Obtain it on the Android build machine with `./gradlew :app:signingReport` inside `android/`. After adding the fingerprint, download the updated `google-services.json` from Firebase and place it at `android/app/google-services.json` on each Android build machine. Git ignores this file. [Android app settings](../android/app/build.gradle.kts) · [Android sign-in setup](https://firebase.google.com/docs/auth/android/google-signin)

## 4. Create the control database and enable Cloud Run

In Firebase **Firestore Database → Create database**, choose **Standard edition**, the **(default)** database, and **Firestore Native mode** where the console offers a choice. For a Vietnam-based pilot, **Singapore (`asia-southeast1`)** is a reasonable candidate. Confirm the location before creation; moving to a different location later requires a migration. Choose **Production mode** and check the **Rules** tab. For a fresh pilot, leave mobile/web access denied until the device identities and per-vehicle access rules are implemented. The current skeleton can run its tests locally without these rules. [Firestore locations](https://firebase.google.com/docs/firestore/locations) · [Firestore database options](https://docs.cloud.google.com/firestore/native/docs/manage-databases)

In Cloud Shell enable the deployment APIs in this project. This prepares source builds and image storage for the future relay. It does not launch a relay or build an image.

```bash
gcloud services enable run.googleapis.com artifactregistry.googleapis.com \
  cloudbuild.googleapis.com firestore.googleapis.com --project="$DASHCAM_PROJECT_ID"
```

## 5. Verify the foundation

```bash
gcloud firestore databases describe --database='(default)' --project="$DASHCAM_PROJECT_ID" \
  --format='table(name,locationId,type,databaseEdition)'
gcloud services list --enabled --project="$DASHCAM_PROJECT_ID" \
  --filter='config.name:(run.googleapis.com OR firestore.googleapis.com OR identitytoolkit.googleapis.com OR artifactregistry.googleapis.com OR cloudbuild.googleapis.com)' \
  --format='value(config.name)'
```

Check the database location and `FIRESTORE_NATIVE`, all five service names, and **Google — Enabled** in Firebase Authentication. Keep the project ID and region for deployment. The current checkout has Jetson-side lifecycle libraries. A public dashboard URL and vehicle enrollment become available after the authenticated relay, the Firestore adapter and the client sign-in integration are implemented. At that stage, attach a dedicated Cloud Run runtime service account, set minimum instances to zero, and add the actual relay hostname to Firebase Authentication's authorized domains. [Cloud implementation plan](CLOUD_MONITORING_PLAN.md)

## 6. Prepare the Android build machine

1. Open a terminal in the repository’s `android/` directory on the computer that produces the APK. Run the signing report for that checkout.

   | Shell | Command |
   |---|---|
   | Linux, macOS or WSL | `./gradlew :app:signingReport` |
   | Windows PowerShell | `.\gradlew.bat :app:signingReport` |

2. Find **Variant: debug** for a local debug APK, or the actual variant used for the test build. Copy its **SHA1** fingerprint. In Firebase, open **Project settings → General → Your apps → com.dashcam.connector → SHA certificate fingerprints**. Add the fingerprint if it is missing and save. Download a fresh **google-services.json** from that Android app and replace the build machine's `android/app/google-services.json`. Git ignores this file. [Firebase Android Google sign-in](https://firebase.google.com/docs/auth/android/google-signin)
3. From the same directory, process the configuration and assemble the debug APK.

   | Shell | Command |
   |---|---|
   | Linux, macOS or WSL | `./gradlew :app:processDebugGoogleServices :app:assembleDebug` |
   | Windows PowerShell | `.\gradlew.bat :app:processDebugGoogleServices :app:assembleDebug` |

   **Pass condition.** Gradle prints `BUILD SUCCESSFUL` and produces `app/build/outputs/apk/debug/app-debug.apk`. A “No matching client found” error means the selected variant’s application ID and downloaded configuration need to be checked together. This build check covers packaging. Interactive Google sign-in requires the client work in step 8.

## 7. Record the LAN baseline and run the existing tests

Keep the phone on the Jetson’s LAN for the current test. Open the paired vehicle in **Dashcam Connector → Connection**. Record the time, app build, vehicle label, whether the app connected, and whether the recorder was running. When the recorder is active, confirm that the dashboard shows fresh recorder data. If the recorder is stopped, record the successful pairing or connection and leave fresh telemetry and preview as pending. A working connection can still report unavailable recorder data.

The following commands inspect the two services independently on the Jetson. Each prints its current state.

```bash
systemctl is-active dashcam-monitor.service
systemctl is-active dashcam-v04.service
```

To check the Plan A libraries offline, run the following from the repository root on a machine with Python 3.10+, GNU Make and a C++17 compiler. These tests use simulated control events and can run alongside the phone test.

```bash
python3 -m unittest discover -s connector -p 'test_cloud_on_demand.py' -v
make -C lib/libclouduplink check
```

**Pass condition.** The Python suite reports `OK` and the native test reports `Plan A uplink lifecycle tests passed`. Record LAN results and library results separately. Neither test opens a Google Cloud session.

## 8. Implement and verify the remote path

This section is for the developer. The current checkout has the lifecycle libraries. Use the following milestones to produce the staging service, enrollment procedure, cloud-enabled APK and dashboard URL. The owner can complete the preceding steps independently.

| Milestone | Work | Evidence before advancing |
|---|---|---|
| 1. Device identity and wake-up | Use simulated devices, then the ARM64 Jetson. Enroll each device with a short-lived code and a revocable identity. Let each read its own `vehicles/{id}/control/current`. Have the authorized server write demand. Exercise token refresh and reconnect. | The device receives its initial state and a new request. Another device cannot read that document. Cached or expired demand starts no upload. |
| 2. Authenticated telemetry relay | Verify the viewer’s Firebase identity and vehicle membership. Aggregate demand across viewers, initially renewing every 10 seconds with a 30-second lease. Once the relay confirms demand, the separate Jetson uplink subscribes to the existing local `/v1/stream`. | An invited viewer receives fresh simulated telemetry. An unassigned user is denied. A second viewer shares the same upstream. Losing the final viewer releases it, including on silent disconnect after timeout and lease expiry. |
| 3. Pilot deployment and clients | Deploy one staging Cloud Run service in Singapore, with request-based billing, minimum zero instances and a pilot maximum of one. Publish its HTTPS URL, configure its Firebase authorized domain and invite one viewer by Firebase UID. Wire the browser and Android cloud connection to the authenticated relay. Install the cloud-enabled APK and enroll the real Jetson. | Both clients can sign in, see only their assigned vehicle and cancel a pending connection. A powered-off vehicle times out. The Jetson continues local recording during relay loss. |
| 4. Camera and the second mode | Forward the existing local preview on explicit camera demand. Then implement **Ready while powered** and the owner’s per-vehicle mode selection. Plan B holds one control connection per powered Jetson with a 60-second heartbeat. | Closing the final camera viewer releases preview while a remaining telemetry viewer continues. In Plan B the heartbeat continues while idle, and silent loss marks the Jetson offline after the planned 150-second timeout. Changing one vehicle’s mode leaves the other vehicle’s session intact. |

An open WebSocket can keep a Cloud Run instance billable. For this occasional-use pilot, start with Plan A, record relay time and outgoing bytes, and compare them with the billing report before expanding to five viewers. Scale-to-zero and a budget alert do not guarantee a zero bill. [Cloud Run WebSockets and billing](https://docs.cloud.google.com/run/docs/triggering/websockets)

The detailed designs cover enrollment and permissions, the lease format, mode changes during a session, and recovery after a relay restart. [Shared architecture](CLOUD_MONITORING_PLAN.md) · [Plan A](CLOUD_ON_DEMAND_PLAN.md) · [Plan B](CLOUD_READY_WHILE_POWERED_PLAN.md)

## 9. First mobile-data acceptance test

Run the telemetry checks once the developer supplies the staging URL, cloud-enabled APK, enrolled vehicle and viewer grant from milestone 3. Use a parked Jetson with working local telemetry. The same Google account can test the app and the web client.

1. Leave the Jetson online. Turn off Wi-Fi on the phone and confirm mobile data works. Open the cloud-enabled app, sign in, select the invited vehicle and tap **Connect**. Repeat in the phone browser using the staging HTTPS URL.
2. Record elapsed time from each tap to the first **fresh recorder sample**. An authenticated vehicle list proves sign-in and access. A fresh sample proves the full path through the relay to the Jetson. Measure the first cold connection separately from a repeat connection. The pilot targets p95 within three seconds across repeated trials.
3. With both viewers connected, close one. The remaining viewer should keep receiving fresh samples. Close the last viewer. Check uplink counters for ten idle minutes. In Plan A, expect zero telemetry and video uploads and a closed device relay socket. Firestore listener traffic is expected.
4. After camera support lands, open the camera and record time to the first fresh frame. Close it and verify the camera reader releases. Check that the LAN app can still connect and that the recorder saved footage during the trial.
5. At the end of the trial, sign out, revoke temporary vehicle grants and stop the separate cloud uplink if returning to LAN-only operation. Revoke the device credential if retiring that enrollment. Confirm a fresh LAN sample and inspect the pilot’s Cloud Run, Firestore and network usage.

| Symptom | First check |
|---|---|
| Google sign-in fails on one APK | Confirm the signing SHA-1 for that exact build and refresh its configuration as in step 6. |
| Sign-in succeeds but the vehicle list is empty | Verify the signed-in Firebase UID has a grant on the intended vehicle in the staging project. |
| Vehicle is listed but Connect times out | Check Jetson power, time and outbound connectivity. Trace the request through the control write, device receipt and relay registration using its request ID. |
| Connected but recorder data is stale | Compare with the LAN baseline and check recorder service state before debugging cloud transport. |

Keep the trial record to build and revision identifiers, timestamps, elapsed times, counters and pass/fail observations. Keep pairing links, identity tokens, coordinates and images out of the record.
