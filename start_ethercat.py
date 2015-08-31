import subprocess
import re
import time

# get the system's network configs
network_configs = subprocess.Popen('/sbin/ifconfig', shell=True, stdout=subprocess.PIPE).stdout.read()

# find out the hardware address
pattern = '^(eth[0-9]*)\s.+?HWaddr\s+([0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2})'

netcard_name_list = []
mac_addr_list = []
for result in re.finditer(pattern, network_configs, flags=re.M):
    netcard_name_list.append(result.group(1))
    mac_addr_list.append(result.group(2))

# ask the user which network mac address will be used
print 'We found ' + str(len(netcard_name_list)) + ' network interface(s) available for EtherCAT configuration\nPlease choose one:'

for i in xrange(len(netcard_name_list)):
    print '[' + str(i) + ']:  ' + netcard_name_list[i] + '  MAC address: ' + mac_addr_list[i]

index_choosed = input('Please input the number')

if index_choosed < 0 or index_choosed > len(netcard_name_list):
    print 'Wrong input, fail to configure EtherCAT, exit...'
    print 'OK'
    exit()

# backup the original ethercat config file
print 'Now backup your original file, we assume you are running this script as root'

origin_config_file = open('/opt/etherlab/etc/ethercat.conf', 'r')
origin_config_text = origin_config_file.read()
origin_config_file.close()

origin_config_backup_file = open('/opt/etherlab/etc/ethercat_backup.conf', 'w')
origin_config_backup_file.write(origin_config_text)
origin_config_backup_file.flush()
origin_config_backup_file.close()

print 'backup OK'

# find the line we need to replace with our new MAC address
pattern = '^(\s*?[^#].*?\")[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}(\")'

result_text = re.sub(pattern, '\g<1>' + mac_addr_list[index_choosed] + '\g<2>', origin_config_text, flags = re.M)


origin_config_file = open('/opt/etherlab/etc/ethercat.conf', 'w')
origin_config_file.write(result_text)
origin_config_file.flush()
origin_config_file.close()

changes = subprocess.Popen('diff /opt/etherlab/etc/ethercat.conf /opt/etherlab/etc/ethercat_backup.conf',
        shell = True,
        stdout = subprocess.PIPE).stdout.read()

print 'The changes just made:'
print changes


# copy the modified files to target places
print 'copy the modified files to target places'

subprocess.Popen('cp /opt/etherlab/etc/ethercat.conf /etc/ethercat.conf', shell = True, stdout = subprocess.PIPE)
subprocess.Popen('cp /opt/etherlab/etc/ethercat.conf /etc/sysconfig/ethercat', shell = True, stdout = subprocess.PIPE)
subprocess.Popen('cp /opt/etherlab/etc/ethercat.conf /opt/etherlab/etc/sysconfig/ethercat', shell = True, stdout = subprocess.PIPE)

print 'EtherCAT configuration finished, start now?'

answer = raw_input('Y/N')

if answer == 'N':
    print 'DONE'
    exit()

if answer == 'Y' or answer == 'y':
    subprocess.Popen('/opt/etherlab/etc/init.d/ethercat start', shell=True)
    time.sleep(1)
    
    subprocess.Popen('/opt/etherlab/etc/init.d/ethercat restart', shell=True)
    time.sleep(9)
    
    subprocess.Popen('/opt/etherlab/bin/ethercat sla', shell = True)
    print 'DONE'

