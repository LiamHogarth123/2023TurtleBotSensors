# 2023TurtleBotSensors


here is a guide to link your git to your linux

first you need to make a ssh key using this website instructions
https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=linux#adding-your-ssh-key-to-the-ssh-agent

Then add it to your github account using this link
https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account?platform=linux


afterwards, you need to clone the information onto your computer
uses this command

```c++
git clone git@github.com:<Name_of_repository>.git
```
That code will download all of the repository
now you need to link username and password
```c++
git config --global user.name "<your_name>"
git config --global user.email "<your_email>"
```
