# Google Cloud integration guide for Dashcam Connect

Use this guide to take one paired Jetson from a working Wi-Fi connection to a telemetry session on a phone using mobile data. Start with **Plan A (On demand)**. The powered Jetson listens for requests in Firestore. When an invited viewer connects, the Jetson opens an outbound connection to the Cloud Run relay. It sends camera frames only if a viewer requests the camera. The planned browser dashboard uses HTTPS and the Android app uses its own cloud connection. Neither client needs a VPN installation or router port forwarding.

## Where this pilot stands — 22 November 2026

| Item | Evidence and next action |
|---|---|
| Google Cloud | The pilot project is `jetson-dashcam-handler`. Cloud Shell confirmed the `(default)` Firestore Native, Standard database in Singapore (`asia-southeast1`) and all five APIs listed in step 5. Reuse this project. Steps 1–5 are also a reference for a fresh installation. |
| Firebase and billing | Web and Android apps are registered, Google sign-in is enabled and the owner completed a monthly billing alert. The project billing link and the staging dashboard's Firebase authorized domain were verified on 22 November. |
| Android build preparation | The owner confirmed step 6 is complete on the Windows APK build machine. The debug signing report succeeded. A Google viewer account signed in on the A54 and received fresh telemetry and camera video through staging over Wi-Fi. |
| Jetson and phone | The owner confirmed the Android phone connects to the Jetson over the LAN. Both Jetson services reported active. All eight Python Plan A tests and the native C++ lifecycle tests passed on the Jetson. |
| Remote viewing | The [staging dashboard](https://dashconnect.cloud.run) is live in Singapore. The signed pilot APK uses the original Cloud Run URL for the same service. ARM64 emulator tests exercised two devices, five viewer sockets, telemetry, camera release and switching modes. Vehicle A is enrolled, and the physical Jetson runs the uplink alongside the LAN monitor. The mobile-data trial is pending. [Pilot handover and APK hash](../HANDOVER.md#2026-11-21-dashcam-connect-cloud-pilot) |

## Prerequisites on your side

| When | What to have ready | How to check |
|---|---|---|
| Now | Access to the existing project in Firebase and Google Cloud, its billing alert recipient and Cloud Shell. | Select the pilot project in both consoles. Steps 2 and 5 give read-only checks for billing and the database. |
| Before building the Android pilot | The computer and signing keystore used for the test APK, Android Studio or an equivalent command-line build environment, a compatible Gradle JDK (JDK 17 is the project baseline), and Android SDK Platform 37. Use the repository's Gradle wrapper. | Open the checkout's `android/` project and complete step 6 on that computer. A different machine may use a different debug keystore. |
| Before testing a real Jetson | Shell access to one powered Jetson, its existing local pairing, a working recorder for the live-data test, and a network with outbound DNS and TLS on TCP 443. The device clock must be accurate for certificates and expiring requests. | Finish the Wi-Fi test in step 7. On the Jetson, `timedatectl status` shows clock and synchronization state. |
| Before first deployment | A deployer who can build and deploy the relay and attach its runtime service account, and an administrator who can grant the build and runtime identities their required access. Cloud Shell is sufficient for the administrator. | Identify who manages project IAM. Use the source-deployment permissions for the chosen build identity. Give the relay a dedicated runtime identity with the Firestore access it needs. [Cloud Run source deployment](https://docs.cloud.google.com/run/docs/deploying-source-code) |
| Before the first external viewing session | One Google account to invite to one vehicle, an Android phone with mobile data and Google sign-in, and a browser with internet access for the web test. | Keep the Jetson on its own internet connection. The phone should be able to turn off Wi-Fi and still reach the internet. One physical Jetson is sufficient for the phone and browser pilot. |

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

Check the database location and `FIRESTORE_NATIVE`, all five service names, and **Google — Enabled** in Firebase Authentication. Keep the project ID and region for deployment. The checkout includes the staging relay and Jetson adapter. Deployment produces the public dashboard URL and enrollment endpoint. At that stage, attach a dedicated Cloud Run runtime service account, set minimum instances to zero, and add the actual relay hostname to Firebase Authentication's authorized domains. [Cloud implementation plan](CLOUD_MONITORING_PLAN.md)

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

To check both Plan A libraries offline on the pilot Jetson, change to the checkout first. Adjust the first line if you cloned it elsewhere. These tests use Python 3.10+, GNU Make and a C++17 compiler and can run alongside the phone test.

```bash
cd ~/Documents/github_repos/dashcam
python3 -m unittest discover -s connector -p 'test_cloud_on_demand.py' -v
make -C lib/libclouduplink check
```

For an optional Windows Python check, open PowerShell in the checkout’s `connector` directory and run the following. The `-s .` argument starts discovery in the current directory. Use `-s connector` only from its parent directory.

```powershell
python -m unittest discover -s . -p 'test_cloud_on_demand.py' -v
```

**Pass condition.** The Python suite reports `OK` and the native test reports `Plan A uplink lifecycle tests passed`. Record LAN results and library results separately. Neither test opens a Google Cloud session.

## 8. Implement and verify the remote path

**Implementation and commands.** The [step 8 runbook](cloud/README.md) covers the emulator test, vehicle grants, staging deployment, Jetson enrollment, Windows APK build and rollback. On the ARM64 development Jetson the emulator demonstrated one telemetry attempt, a 30.05-second silent release, a 0.06-second explicit release, denial of cross-vehicle reads and device writes, and loss of access after identity rotation. The TLS integration test used the existing LAN connector with synthetic recorder data. Live staging and LTE acceptance still require the project deployer, registered Windows signing key and phone.

**Your starting point.** You have completed Android build preparation and verified phone-to-Jetson connectivity over Wi-Fi. Step 8 builds the software that carries that same read-only telemetry over the internet. The existing Python and C++ tests cover how the Jetson reacts to demand. They run without Firebase. The first new test below exercises a real listener against the local Firestore emulator.

| Who and where | First action | What you should receive |
|---|---|---|
| Developer, on the development PC | Build the emulator test in 8.1, then the listener, enrollment flow and relay. | A repeatable test run showing an authorized wake-up, access denied for the other identity, and automatic release on expiry. |
| You, on the PC and in Firebase | When staging is ready, choose the Google account to invite. The developer or project administrator sets the staging hostname and grants that account access to vehicle A. | An HTTPS dashboard URL, a cloud-enabled debug APK, and the vehicle label to select. |
| You and the developer, on the Jetson | When the uplink is ready, enroll the Jetson once using the supplied command and a short-lived code. Keep the existing LAN pairing. | An enrolled vehicle that keeps recording locally while awaiting a viewer. |
| You, on the phone | After the staging handoff in 8.4, use mobile data for the telemetry checks in step 9. | Fresh samples from the enrolled Jetson in the browser and Android app. Camera and the mode toggle follow in 8.5. |

### 8.1 Prove the wake-up and permissions on the development PC

1. **Create the local test environment.** The developer adds a Firebase Emulator Suite configuration for Authentication and Firestore, explicit Firestore rules, a small data setup script, and an automated listener test. Use a demo project ID such as `demo-dashcam` throughout the test. Run the emulators on loopback. Keep emulator settings confined to the test process. Once those files and the Firebase CLI are in place, the developer can start both emulators from the directory containing `firebase.json` with `firebase emulators:start --only auth,firestore --project demo-dashcam`. [Firebase emulator setup](https://firebase.google.com/docs/emulator-suite/connect_firestore)
2. **Represent two vehicles with two processes.** Give simulated A and B distinct authenticated device identities. Give the test viewer access to A. Use a trusted test writer to create the control documents. Make all device reads through the rules-enforced client path. Firestore server libraries use IAM and bypass these rules, so an administrator read cannot establish device isolation. [Firestore authorization](https://firebase.google.com/docs/firestore/security/get-started)
3. **Start at idle.** A subscribes to `vehicles/A/control/current`. Its initial server-confirmed snapshot has `desiredMode=on_demand`, `configVersion=1`, `demandGeneration=0`, `channels=[]`, and `expiresAt=null`. The listener should stay connected with zero relay attempts and zero media readers.
4. **Request telemetry once.** The test writer advances the generation to 1, sets `channels=["telemetry"]` and writes a Firestore timestamp 30 seconds in the future. The listener supplies the actual document path, fields and snapshot metadata to the lifecycle library. Expect one outbound connection attempt. Use a simulated relay acknowledgement for this test. Only after it confirms the same attempt and generation should the telemetry reader become active. Video stays idle.
5. **Prove release and isolation.** First withhold renewal and verify that the reader and relay attempt end at the local lease deadline. Then send a fresh request and explicitly clear demand by advancing the generation and emptying the channel list. Verify immediate release. B must successfully read B’s document and receive permission denied for A’s document. Reject a device write to its own demand record as well. Reconnect A after a lease expires and verify that cached or expired demand leaves it idle.

**Evidence to keep:** the observed generation, connection attempts, active reader counts, permission results and the time to release. A single local run should reproduce all of them. The developer should deliver the test command with the emulator configuration. This milestone uses synthetic data and needs no phone or cloud deployment.

### 8.2 Enroll the Jetson and connect the native lifecycle library

1. Implement an owner-issued, single-use enrollment code with a short expiry. Exchange it over TLS for a revocable identity bound to vehicle A. Store the device credential in a file readable only by the uplink service account. Document how to recover if the exchange succeeds but the Jetson loses the reply.
2. Run the uplink as a separate host process. Its cloud connection uses outbound TLS. For local telemetry it uses the already-paired connector and verifies its certificate. Keep the recorder and LAN connector running normally. Install the cloud uplink on the pilot Jetson when a staging enrollment endpoint is available in 8.4.
3. Feed device-scoped listener events into the C++ controller on its owning event loop. Follow the target it returns to open or close the socket and readers. Arm its monotonic deadline even while the network handshake is pending. Refresh credentials, reconnect with backoff, and obtain a current server snapshot before resuming. [Native adapter interface](../lib/libclouduplink/README.md)

**Pass on ARM64:** with synthetic demand, observe one wake-up after a current update. Reconnect after an expired request and observe zero readers. Revoke the test identity and verify that access ends. Repeat enrollment with a new credential before the live pilot.

### 8.3 Build the relay and connect the two clients

The relay links a signed-in viewer’s request to the enrolled Jetson. The implementation exposes the following paths. A staging deployment is required for the phone trial.

| Event | What the implementation must do | Observable result |
|---|---|---|
| Sign in and list vehicles | Verify Firebase identity and look up vehicle membership by UID. Serve the browser from the relay origin with a secure session cookie. The Android app uses a short-lived ID token and the default network for cloud traffic. | The invited account sees A. An account with no grant sees no vehicle and cannot subscribe by supplying A’s ID. [Firebase token verification](https://firebase.google.com/docs/auth/admin/verify-id-tokens) |
| Tap Connect | Create a bounded viewer session. In a transaction, aggregate unexpired demand for A and update its control document. Renew active leases initially every 10 seconds for a 30-second lifetime. | Firestore wakes A as soon as demand changes. The UI shows **Connecting**. Cancelling an offline request releases the pending session. |
| Device registers | Authenticate A on the proposed `/v1/uplink` socket and recheck current demand and generation. Open one authenticated local `/v1/stream` reader only after confirmation. Fan its snapshots out to authorized viewers. | Two viewers on A receive advancing sequence numbers from one upstream. Show **Live** when recorder data is fresh. A powered Jetson with an idle recorder can report that separately. |
| Leave, lose connectivity or lose access | On explicit leave, remove only that viewer’s demand. On silent loss, bound upload by viewer timeout plus the remaining lease. Recheck authorization for open sessions and close revoked ones. | Closing the browser keeps the phone live. Closing the final viewer releases A’s telemetry reader and Plan A relay socket. Local recording continues. |

For a failure, follow one request ID from the tap through authorization, Firestore write, device receipt, relay confirmation and first fresh sample. Record elapsed time on one clock for the user-visible measurement. Keep tokens, location and frame contents out of the logs.

### 8.4 Deploy one staging relay and hand it over for the phone test

1. **Developer and cloud administrator:** package the tested relay, Firestore rules, client configuration and uplink. Target the existing project `jetson-dashcam-handler`, default database and region `asia-southeast1`. Use a dedicated runtime service account. Bind vehicle access in the application before making the dashboard reachable from the internet.
2. **Developer:** deploy one staging Cloud Run service with request-based billing, minimum zero instances and a pilot maximum of one. Configure the WebSocket request timeout and test reconnection across timeout or a new revision. An open WebSocket keeps an instance active for billing. Record relay time and outgoing bytes during the trial. [Cloud Run WebSockets and billing](https://docs.cloud.google.com/run/docs/triggering/websockets)
3. **Firebase administrator:** add the actual staging hostname under **Authentication → Settings → Authorized domains**. Have the selected user sign in, then grant its Firebase UID the intended role for A using the implemented invitation or grant procedure. Confirm that the same account sees its vehicle on both clients.
4. **Developer and owner:** enroll A using the delivered uplink, install the cloud-enabled APK signed with the certificate registered in step 6, and open the staging URL in the browser. Test an offline request and cancellation, then bring the uplink online and verify delivery from the real recorder. Use the local dashboard for comparison if samples appear stale.
5. **Handoff:** provide the exact HTTPS URL, APK build identifier, Jetson enrollment and uplink stop instructions, pilot vehicle label and staged relay revision. These values come from the actual deployment. With both clients able to select A, run the mobile-data telemetry checks in step 9.

**Pass for the first remote session:** with phone Wi-Fi off, both the Android app and browser can receive a fresh recorder sample from A. After the last viewer exits, monitor ten idle minutes. Plan A should have zero telemetry/video uploads, a closed device relay socket and only control-listener or configured presence traffic. If the trial needs to stop, stop the separate uplink, remove the temporary grant and verify the LAN connection.

### 8.5 Add video and the selectable second mode

- **Camera.** Open the existing local `/v1/video` reader on the first explicit camera request. Share one upstream across viewers. Close the reader after the last camera viewer leaves, even if someone is still watching telemetry. Verify first-frame latency separately and confirm the Jetson records through a cloud disconnect. [Local video protocol](README.md#video-protocol)
- **Ready while powered.** Add the owner setting after Plan A telemetry works. The device opens its control socket on boot or network recovery, sends a 60-second heartbeat and becomes offline after the planned 150-second silent-loss deadline. Keep media readers closed until requested. Test switching during viewing and again with zero viewers.
- **Pilot capacity.** Use the real Jetson for recording, LTE and timing. Use a second authenticated simulator for vehicle B. Split up to five viewer sessions across A and B, change A’s mode, and confirm that B’s subscriptions and data stay independent. Compare first-sample latency and billable relay time before choosing a per-vehicle default. [Shared architecture](CLOUD_MONITORING_PLAN.md) · [Plan A](CLOUD_ON_DEMAND_PLAN.md) · [Plan B](CLOUD_READY_WHILE_POWERED_PLAN.md)

## 9. First mobile-data acceptance test

Run the telemetry checks once the developer supplies the staging URL, cloud-enabled APK, enrolled vehicle and viewer grant from step 8.4. Use a parked Jetson with working local telemetry. The same Google account can test the app and the web client.

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
