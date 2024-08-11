import requests
import time

def ping(url):
  start_time = time.time()
  response = requests.get(url)
  end_time = time.time()

  latency = end_time - start_time
  return latency
