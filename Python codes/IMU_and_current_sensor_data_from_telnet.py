from pixelinkWrapper import*
import os
import time
import msvcrt
import glob
import grequests
import requests
import concurrent.futures
import numpy as np
from telnetlib import Telnet

t_initial = time.time()
print(f'Telnet-{t_initial} s.......')


url = []
# ip_address = ['111']
ip_address = ['103', '115', '132', '155', '143', '125', '180', '174', '164', '111', '114', '192', '160', '197', '184', '113', '175', '126', '122', '172']
for ip in ip_address:
    url.append("http://192.168.0."+ip+"/start")

req = (grequests.get(u) for u in url)
res = grequests.map(req)
# for r in res:
#     print(r.text)
#%

port = 23
Dest_folder_telnet_data = 'E:\\Soumen\\Squiggle Balls\\Telnet data Final\\RSI Squigglebot different orientation\\Telnet data\\'

if not(os.path.isdir(Dest_folder_telnet_data)):
    os.makedirs(Dest_folder_telnet_data)
else:
    data_files = glob.glob(Dest_folder_telnet_data+'\\*')
    if len(data_files) != 0:
        for data_file in data_files:
            os.remove(data_file)

count = 400
# notquit = True
flag = True
tn = [Telnet('192.168.0.'+ip,port) for ip in ip_address]
time.sleep(5 - (time.time() - t_initial))

    
def comm(ip_address, tn):
    datas=[]
    print(f'Telnet from 192.168.0.{ip_address} -{time.time()} s.......')    
    k = 0
    elapsed_time = 0
    current_time = 0
    t = time.perf_counter()
    previous_time = t
    fid = open(Dest_folder_telnet_data+"\\"+ip_address+".txt", "a+")
    # try:
    while(flag):
        k+=1
        # t = time.perf_counter()
        data = tn.read_until(b'\r\n', timeout=5)
        current_time = time.perf_counter()
        elapsed_time += (current_time - previous_time)
        data_str = data.decode('utf-8')
        d = data_str.split()
        if (len(d)==9):
            data_str = '\n' + f'{elapsed_time:.3f}' + '\t' + data_str
            fid.write(data_str)
        else:
            tn.close()
            try:
                r = requests.get("http://192.168.0."+ip_address+"/start", timeout=(10, 5))
                print(r.url)
                print(r.text)                                
                tn = Telnet('192.168.0.'+ip_address, port)
                data = tn.read_until(b'\r\n', timeout=5)
                current_time = time.perf_counter()
                elapsed_time += (current_time - previous_time)
                data_str = data.decode('utf-8')
                d = data_str.split()
                data_str = '\n' + f'{elapsed_time:.3f}' + '\t' + data_str
                if (len(d)==9):
                    fid.write(data_str)
                else:
                    fid.close()
                    tn.close()
                            
            except Exception:
                # print(f'Connection timed out')
                print(Exception)
                break
        previous_time = current_time
    print(f'192.168.0.{ip_address} - time passed : {time.perf_counter()-t} s')
    fid.close()
    tn.close()
    print(k)
    return(datas, ip_address, k, f'time passed : {time.perf_counter()-t} s')
        

try:
    
    t1 = time.perf_counter()
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=None)
    results = []
    for i in range(len(ip_address)):
        results.append(executor.submit(comm, ip_address[i], tn[i]))
    
    notquit = True
    
    while(notquit):
        checkflag = False
        
        for j in range(len(results)):
            checkflag = checkflag or not(results[j].done())
        # for result in results:
        #     checkflag = checkflag or not(result.done())
            
        # pass
        notquit = checkflag 
    print('All the workers stopped')
except Exception:
    print(Exception)    
except KeyboardInterrupt:
    flag = False
    print("Keyboard interrupt")
finally:
    flag = False
    # executor.shutdown(wait=False)
    print('stopping the program')
    future_return = []
    # for result in concurrent.futures.as_completed(results):
    #     future_return.append(result.result())
   
t2 = time.perf_counter()


print('Finished in ', t2-t1, ' seconds')

file_id = open('E:\\Soumen\\Squiggle Balls\\Python codes\\finished.txt', "w+")
file_id.write('Gyro data taking finished')
file_id.close()