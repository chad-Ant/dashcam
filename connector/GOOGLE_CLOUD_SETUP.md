# Google Cloud setup for Dashcam Connect

This procedure prepares a pilot project for two Jetsons and five viewers. Both cloud connection modes use the same project, Firestore database and user identities. Use a browser on your own computer for the administrator steps. The Jetsons will get individual device credentials during enrollment.

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

In [Billing](https://console.cloud.google.com/billing), create or select a billing account and link **only the intended pilot project**. Google handles payment details directly. In **Budgets & alerts**, create a monthly budget scoped to the dashcam project. For a low-cost trial, a starting alert amount is **US$5** or your local equivalent, with thresholds at 50%, 90%, and 100%. Choose an email recipient you check. An alerts-only budget emails you about spend; charges can continue after a threshold. Confirm the link in Cloud Shell. [Budget documentation](https://docs.cloud.google.com/billing/docs/how-to/budgets)

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
