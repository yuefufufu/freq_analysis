#!/bin/bash

# ディレクトリを指定
dir="/home/yue/catkin_ws/rosbag"
target_dir="/home/yue/catkin_ws/rosbag/rosbag"

# ディレクトリ内の各ファイルに対してループ処理
for filepath in $dir/*.bag; do
  # ファイル名を取得
  filename=$(basename "$filepath")
  
  # ファイル名が指定されたパターンに一致するか確認
  if [[ $filename =~ ^(.*)\_([0-9]{3})\.bag$ ]]; then
    # ベース名と番号を取得
    base_name=${BASH_REMATCH[1]}
    number=${BASH_REMATCH[2]}
    
    # カウンターを初期化
    counter=0
    
    # 既存のファイル名が存在する間、カウンターをインクリメント
    while [[ -e "$dir/${base_name}_${number}-$counter.bag" ]] || [[ -e "$target_dir/${base_name}_${number}-$counter.bag" ]]; do
      ((counter++))
    done
    
    # ファイル名を変更し、目的のディレクトリに移動
    mv "$filepath" "$target_dir/${base_name}_${number}-$counter.bag"
  fi
done

