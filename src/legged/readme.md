# raisim
'''
echo "/home/jing/Documents/raisimLib/raisimUnityOpengl/linux/raisimUnity.x86_64" >> ~/araisim
'''

# generate ssh
'''
ssh-keygen -t ed25519 -C "liangjingjerry@gmail.com"
'''
press "Enter" and then type in the password

start ssh-agent in background:
'''
eval "$(ssh-agent -s)"
'''

add ssh key to ssh-agent:
'''
ssh-add ~/.ssh/id_ed25519
'''
