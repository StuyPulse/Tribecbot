#!/usr/bin/python3
import os
from fabric import Connection

MAX_NUMBER_OF_LOGS_TO_PULL = 5
TARGET_IP_ADDRESS = "10.6.94.2"
USB_DIRECTORY = "D:\\MILSTEIN_LOGS"
LOCAL_DIRECTORY_TO_SAVE_LOGS = "C:\\Users\\test\\logs\\CHAMPS_LOGS_26"
os.makedirs(LOCAL_DIRECTORY_TO_SAVE_LOGS, exist_ok=True) # if the dir does not exist, create a new one

SAVE_DIRECTORY = USB_DIRECTORY if os.path.exists(USB_DIRECTORY) else LOCAL_DIRECTORY_TO_SAVE_LOGS

raw_local_logs = os.listdir(path=SAVE_DIRECTORY) 

print("DIRECTORY BEING WRITTEN TO: " + SAVE_DIRECTORY)
print(f"THESE FILES WERE ALREADY FOUND FROM {SAVE_DIRECTORY}: " + str(raw_local_logs))

wpilogs_local = set(filter(lambda x: x.endswith(".wpilog"), raw_local_logs))

ssh_connection = Connection(
    host=TARGET_IP_ADDRESS,
    user="lvuser",
    connect_kwargs={"password": ""},
    connect_timeout=4
)

raw_remote_logs = ssh_connection.run("ls -t /home/lvuser/logs/", hide=True)
remote_files = raw_remote_logs.stdout.strip().split()

all_remote_wpilog_logs = [f for f in remote_files if f.endswith(".wpilog")]
wpilogs_remote_newest = all_remote_wpilog_logs[:MAX_NUMBER_OF_LOGS_TO_PULL]

logs_to_be_downloaded = []
for log_name in wpilogs_remote_newest:
    if log_name not in wpilogs_local:
        print(f"Missing log found, appending {log_name} to {SAVE_DIRECTORY}" )
        logs_to_be_downloaded.append(log_name)
        ssh_connection.get(f"/home/lvuser/logs/{log_name}", local=f"{SAVE_DIRECTORY}/{log_name}") # need to include the name of the file
        if (os.path.exists(SAVE_DIRECTORY)):
            print(f"SUCCESS: {SAVE_DIRECTORY} found and being written to")
        else:
            print(f"ERROR: {SAVE_DIRECTORY} not found!")

    
#todo: add a post request to an api endpoint that allows us to automatically upload logs to a server

print(f"Downloaded {len(logs_to_be_downloaded)} log(s): {logs_to_be_downloaded} directory: {SAVE_DIRECTORY} ")
