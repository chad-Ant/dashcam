FROM python:3.11-slim
WORKDIR /app/cloud
COPY cloud/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY cloud/store.py cloud/relay.py ./
COPY cloud/web/ web/
COPY web/ /app/web/
USER 65532
CMD ["python", "-u", "relay.py"]
