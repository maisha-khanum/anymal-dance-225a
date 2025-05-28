import redis
r = redis.Redis(host='192.168.0.234', port=6379)
print("Type a number (positive or negative) and press Enter to send it:")
while True:
    key = input("Number: ").strip()
    if key.lstrip('-').isdigit():
        print(f"Sending: {key}")
        r.publish('direction', key)
    else:
        print("Invalid input. Please enter a valid integer.")