FROM python:3.10-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the simulator code
COPY . .

# Environment variables for headless output
ENV OUTPUT_PATH=/app/data/flight_plot.png

# Run the simulator
CMD ["python", "main.py"]
