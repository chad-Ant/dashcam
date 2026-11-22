# Step 8 staging and verification

The relay serves a Google sign-in dashboard and accepts Firebase ID tokens from the Android cloud screen. The Jetson runs a separate uplink process. Its device identity can read one Firestore document. It checks the pinned LAN server certificate before forwarding the existing telemetry or DCV1 camera stream. Vehicle IDs such as `A` are cloud identifiers. The local pairing carries the recorder's separate device ID.

## Repeat the local test

From the repository root on Linux or WSL, use Python 3.10+, a C++17 compiler, Make, Node 22, Java 21 and the Firebase CLI (tested with 14.26.0). The device requirements are a subset of the emulator dependencies. Use a fresh emulator invocation to start each run with an empty database.

```sh
cd connector/cloud
python3 -m venv /tmp/dashcam-cloud-test
. /tmp/dashcam-cloud-test/bin/activate
pip install -r requirements.txt -r device-requirements.txt
make
firebase emulators:exec --project demo-dashcam --only auth,firestore \
  'python tests/emulator.py && python tests/relay_path.py'
```

The first script seeds two identities, launches two device processes and logs reader masks (1 = telemetry, 2 = video), attempt numbers, permission checks and release times. The second script starts a TLS relay and two TLS LAN connectors with synthetic recorder inputs, then exercises five viewer sockets. It uses both a browser session cookie and bearer tokens. All emulator settings are scoped to those processes.

## Prepare the staging project

Use an administrator/deployer shell with this checkout and the credentials for `jetson-dashcam-handler`. In the Firebase console, select the **web** registration under **Project settings → Your apps → SDK setup and configuration**. Export the configuration object as valid JSON in `FIREBASE_WEB_CONFIG`. It includes `projectId`, `authDomain`, `appId` and `apiKey`. The relay uses its attached service account for privileged operations. Keep service-account keys out of this variable.

Create or reuse the dedicated runtime identity. The following are one-time provisioning commands for new resources. The custom role limits the Firebase Auth calls to account lookup and session-cookie creation. Signing is scoped to the relay account itself. The deployer also needs permission to deploy from source and attach that account as described in step 5.

```sh
project=jetson-dashcam-handler
account=dashcam-relay@"$project".iam.gserviceaccount.com
gcloud services enable iamcredentials.googleapis.com --project="$project"
gcloud iam service-accounts create dashcam-relay --project="$project"
gcloud iam roles create dashcamRelayAuth --project="$project" --title='Dashcam relay authentication' \
  --permissions=firebaseauth.users.get,firebaseauth.users.createSession
gcloud projects add-iam-policy-binding "$project" --member="serviceAccount:$account" --role="projects/$project/roles/dashcamRelayAuth"
gcloud projects add-iam-policy-binding "$project" --member="serviceAccount:$account" --role=roles/datastore.user
gcloud iam service-accounts add-iam-policy-binding "$account" --project="$project" \
  --member="serviceAccount:$account" --role=roles/iam.serviceAccountTokenCreator
```

The staging source build uses a separate service account to read its uploaded sources, write build logs and push the image. For a new project, provision it once. The deployer needs permission to attach both service accounts. [Cloud Run build service accounts](https://docs.cloud.google.com/run/docs/configuring/services/build-service-account)

```sh
builder=dashcam-relay-build@"$project".iam.gserviceaccount.com
gcloud iam service-accounts create dashcam-relay-build --project="$project"
gcloud projects add-iam-policy-binding "$project" \
  --member="serviceAccount:$builder" --role=roles/run.builder
```

From `connector/cloud`, run `./stage.sh` after comparing the current default-database rules with `firestore.rules`. The deployment publishes that ruleset for the entire default database. Preserve any separately managed rules if the project is shared. The script checks billing, deploys the single-instance relay, then prints the actual service URL and revision. The pilot uses an in-memory connection registry, one active revision receiving all traffic, and one instance. Stop old sessions before shifting revisions. Cloud Run terminates TLS at its ingress.

Add the printed hostname to **Firebase Authentication → Settings → Authorized domains**. Open the printed URL and sign in once with the chosen Google account. A new account sees an empty vehicle list. In **Authentication → Users**, copy that account's **User UID**. This identifies the person who can view vehicle A and issue its enrollment code. The label below is the name displayed to that person. Run the grant from an administrator Python environment with the relay requirements installed and application default credentials for the pilot project.

```sh
read -rp 'Pilot owner Firebase UID: ' pilot_uid
python admin.py --project jetson-dashcam-handler grant A --uid "$pilot_uid" --role owner --label 'Pilot Jetson'
```

Refresh the dashboard. For additional people, use their individual UIDs and `--role viewer`. The owner selects **Issue enrollment code**. Each code expires in five minutes and permits one exchange. Issuing a newer code supersedes the previous pending code.

## Install and enroll one Jetson

Run from `connector/cloud` on the Jetson. The service needs outbound HTTPS and HTTP/2 to Firestore. Use the paired LAN connector's existing link. The installer copies only the uplink and its runtime into `/opt/dashcam-uplink`.

```sh
make
id dashcam-uplink >/dev/null 2>&1 || sudo useradd --system --user-group --no-create-home --shell /usr/sbin/nologin dashcam-uplink
sudo install -d /opt/dashcam-uplink
sudo install -m 0644 uplink.py listener.py native.py libdashcam_uplink.so /opt/dashcam-uplink/
sudo python3 -m venv /opt/dashcam-uplink/venv
sudo /opt/dashcam-uplink/venv/bin/pip install -r device-requirements.txt
sudo install -d -m 0700 -o dashcam-uplink -g dashcam-uplink /var/lib/dashcam-uplink
read -rp 'Staging HTTPS origin: ' relay_origin
sudo -u dashcam-uplink /opt/dashcam-uplink/venv/bin/python /opt/dashcam-uplink/uplink.py enroll \
  --relay "$relay_origin" --state /var/lib/dashcam-uplink/device.json
sudo install -m 0644 dashcam-uplink.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now dashcam-uplink.service
```

The enrollment prompt hides the LAN pairing link and cloud code. The private state file holds the refresh token and the separate local pairing, owned by `dashcam-uplink` with mode 0600. If an exchange succeeds but its reply or state file is lost, revoke the device binding with the command below, issue a new code, repeat enrollment and restart the service. Successful replacement exchanges rotate the device UID atomically. The previous UID loses its Firestore and relay binding.

## Build the APK and verify the handoff

On the Windows machine from step 6, open PowerShell in `android` and enter the exact HTTPS origin printed by the deployment. The registered debug certificate must match this machine's key.

```powershell
$relayOrigin = Read-Host 'Staging HTTPS origin (no trailing slash)'
.\gradlew.bat "-PdashcamRelayOrigin=$relayOrigin" :app:assembleDebug
Get-FileHash .\app\build\outputs\apk\debug\app-debug.apk -Algorithm SHA256
```

Install that APK and open **Connection → Cloud vehicles**. Sign in, select the granted vehicle and connect. The Android cloud connection uses the default network. Connect the browser independently. Pass the actual APK hash, deployed revision, HTTPS origin and vehicle label to the tester.

With the Jetson parked, run the mobile-data and ten-minute idle checks in step 9 of the setup guide. Test an offline connection and cancellation first. Time telemetry and camera startup separately, then roll a revision or reach the WebSocket timeout during a session and check for a new fresh sequence. Cloud Run's billable instance time and network egress metrics cover the staging cost trial. The relay emits request IDs at authorization, demand commit, registration and first fresh telemetry. The uplink journal records demand receipt. Check the recorder's saved footage across a disconnect.

To return to LAN-only operation or retire an enrollment:

```sh
# Jetson
sudo systemctl disable --now dashcam-uplink.service
# Administrator shell in connector/cloud
python admin.py --project jetson-dashcam-handler remove-grant A --uid "$pilot_uid"
python admin.py --project jetson-dashcam-handler revoke-device A
```

Recheck the local dashboard and recorder after the trial.
