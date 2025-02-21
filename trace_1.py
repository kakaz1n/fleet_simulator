import requests
# Replace with your MiR100 API base URL and credentials
BASE_URL = "http://10.83.131.155/api/v2.0.0"
HEADERS = {
    "accept": "application/json",
    "Authorization": "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA==",
    "Accept-Language": "en_US"
}

# Step 1: Get the list of missions
missions_response = requests.get(f'{BASE_URL}/missions', headers=HEADERS)
missions = missions_response.json()
# print(missions)
# Assuming you want the latest mission
latest_mission_id = missions[0]['guid']

# Step 2: Get the actions for the latest mission
actions_response = requests.get(f'{BASE_URL}/missions/{latest_mission_id}/actions', headers=HEADERS)
actions = actions_response.json()

# Step 3: Extract the path trace
path_trace = []
for action in actions:
    if action['action_type'] == 'move':
        path_trace.extend(action['path'])

# Now `path_trace` contains the trace of the path
print(path_trace)