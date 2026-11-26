#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
project=jetson-dashcam-handler
region=asia-southeast1
service=dashcam-relay-staging
account=dashcam-relay@"$project".iam.gserviceaccount.com
builder=dashcam-relay-build@"$project".iam.gserviceaccount.com
# Cloud recordings: the bucket clips go to (README "Cloud recordings bucket"). CLIP_BUCKET= (empty) deploys without clips.
clip_bucket=${CLIP_BUCKET-$project-clips}
: "${FIREBASE_WEB_CONFIG:?Set FIREBASE_WEB_CONFIG to the registered web app configuration as JSON}"
envfile=$(mktemp)
trap 'rm -f "$envfile"' EXIT
python3 - "$envfile" "$project" "$clip_bucket" <<'PY'
import json, os, sys
config = json.loads(os.environ['FIREBASE_WEB_CONFIG'])
if config.get('projectId') != sys.argv[2] or not all(config.get(k) for k in ('apiKey','authDomain','appId')):
    sys.exit('Expected the registered Firebase web configuration for the pilot project')
env = {'FIREBASE_WEB_CONFIG':json.dumps(config)}
if sys.argv[3]:
    env['CLIP_BUCKET'] = sys.argv[3]
with open(sys.argv[1], 'w') as f:
    json.dump(env, f)
PY
gcloud iam service-accounts describe "$account" --project="$project" >/dev/null
gcloud iam service-accounts describe "$builder" --project="$project" >/dev/null
[[ $(gcloud billing projects describe "$project" --format='value(billingEnabled)') == True ]]
[[ -z $clip_bucket ]] || gcloud storage buckets describe gs://"$clip_bucket" --format='value(name)' >/dev/null
firebase deploy --only firestore:rules --project "$project"
gcloud run deploy "$service" --project="$project" --region="$region" --source=.. \
  --build-service-account="projects/$project/serviceAccounts/$builder" \
  --service-account="$account" --env-vars-file="$envfile" --cpu-throttling \
  --min=0 --max=1 --max-instances=1 --concurrency=40 --timeout=3600 --allow-unauthenticated
gcloud run services describe "$service" --region="$region" --project="$project" \
  --format='table(status.url,status.latestReadyRevisionName)'
