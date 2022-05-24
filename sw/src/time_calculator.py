server_out = open("server_out.txt", "r")
server_lines = server_out.readlines()
server_out.close()

client_out = open("client_out.txt", "r")
client_lines = client_out.readlines()
client_out.close()

sent_lines = list(filter(lambda line: "Sent at" in line, client_lines))
recv_lines = list(filter(lambda line: "Received at" in line, server_lines))

if len(sent_lines) == len(recv_lines):
    pass
else:
    print(f"#SentLines ({len(sent_lines)}) != RecvLines ({len(recv_lines)})")
    exit(1)