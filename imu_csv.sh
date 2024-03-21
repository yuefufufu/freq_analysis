#!/bin/bash

# 現在のディレクトリにあるすべての .bag ファイルをループ処理
for bagfile in *.bag
do
  # ファイル名から拡張子を除去
  filename=$(basename "$bagfile" .bag)
  
  # ファイル名を分解して変数に格納
  IFS='_' read -ra ADDR <<< "$filename"
  prefix="${ADDR[0]}"
  num1="${ADDR[1]}"
  num2="${ADDR[2]}"

  # rostopic コマンドを実行
  if ! rostopic echo -b "$bagfile" -p /imu/data_raw > "${prefix}_imu_${num1}-${num2}.csv"; then
    echo "Error processing file: $bagfile"
  fi
done

