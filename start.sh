#!/bin/bash


cmd=$1
if [ "$*" = 0 -o "$cmd" != run ]; then
  echo "call $0 [run]"
  exit -1
fi

function info () {
  echo "command: $0"
  echo "baseDir: $baseDir"
}


function genCert () {
  local dir="$1"
  local domain="$2"
  local subject="${3/\$domain/$domain}"
  pushd $dir
  #eval 'subject="'$subject'"'
  echo "domain='$domain'"
  echo "subject='$subject'"
  RANDFILE=/tmp/.rnd openssl req -nodes -newkey rsa:2048 -keyout ssl_key.pem -out ssl.csr -subj "$subject"
  RANDFILE=/tmp/.rnd openssl x509 -in ssl.csr -out ssl_cert.pem -req -signkey ssl_key.pem -days 1001
  popd
}


baseDir="${0%%/*}"
if [[ "$baseDir" != /* ]]; then baseDir="$(pwd)/$baseDir"; fi
baseDir="${baseDir%%/.}"

env=$(mktemp)
$baseDir/web.py env $baseDir/config.yaml > $env
source $env

info
cat $env

if ! [ -e $baseDir/config ]; then
  mkdir $baseDir/config
  mkdir $baseDir/log
  mkdir $baseDir/web
  if ! [ -r $baseDir/config/cert.pem ]; then
    genCert $baseDir/config $web__domain "$web__cert_subject"
    cat $baseDir/config/ssl_{key,cert}.pem > $web__cert_path
  fi

fi

