FROM python:3.11-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY store.py relay.py ./
COPY web/ web/
USER 65532
CMD ["python", "-u", "relay.py"]
