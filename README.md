# Event camera's and their required software

## Git initialization

### ssh key creation

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"

```
Hit enter to put it in ~/.ssh/id_ed25519
Make a passphrase that you will remember to access it
```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519

```
Enter the passphrase

```bash
cat ~/.ssh/id_ed25519.pub
 
```
copy the output of that command and paste it in github.com

1. click on profile 
2. go to accessibility
3. go to SSH and GPG keys
4. click add key and name it something
5. paste the key that you copied from shell terminal

```bash
# test connection
ssh -T git@github.com

```
If connection successful, clone repo

```bash
git clone git@github.com:Relativity1395/evnt_work.git

```

## Installing dependancies for git repository

```bash
git clone git@github.com:Relativity1395/evnt_work.git

sudo apt-get install build-essential cmake pkg-config libusb-1.0-0-dev

cd libcaer

cmake -DCMAKE_INSTALL_PREFIX=/usr .

make

sudo make install

```
