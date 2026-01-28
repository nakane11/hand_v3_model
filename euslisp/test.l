#!/usr/bin/env roseus

;; --- 1. クラス定義 ---

(defclass hand-holding
  :slots ())

(defmethod hand-holding
  (:init ()
   (unless (ros::ok) (ros::roseus "hand_holding"))
   (ros::advertise "hand_angle" std_msgs::Float32MultiArray 1)
   (ros::advertise "classifier_cmd" std_msgs::String 1)
   (format t "[System] Hand Interface Initialized.~%")
  )

  ;; --- 共通メソッド ---
  (:publish-angle (angle-vec)
   (let ((pub-msg (instance std_msgs::Float32MultiArray :init)))
     (send pub-msg :data angle-vec)
     (ros::publish "hand_angle" pub-msg)))

  (:send-classify-cmd (cmd-str)
   (let ((msg (instance std_msgs::String :init)))
     (send msg :data cmd-str)
     (ros::publish "classifier_cmd" msg)
     (format t ">> [System] Sent Command: '~A'~%" cmd-str)))

  (:detect-hand-class ()
   (format t ">> [Step 1] Start Recognition...~%")
   ;; 1. 推論モードON
   (send self :send-classify-cmd "s")

   (let ((history nil)
         (looping t)
         (detected-id nil)
         (counts (instantiate integer-vector 7)))

     (while (and (ros::ok) looping)
       ;; キャンセルチェック
       (when (select-stream (list *standard-input*) 0.001)
         (let ((c (read-char *standard-input*)))
           (when (or (eq c #\q) (eq c #\c))
             (send self :send-classify-cmd "s")
             (format t ">> Canceled.~%")
             (return-from :detect-hand-class nil))))

       ;; データ受信と多数決判定
       (let ((msg (one-shot-subscribe "hand_class" std_msgs::Int8)))
         (when msg
           (let ((val (send msg :data)))
             (push val history)
             (if (> (length history) 10) (setq history (subseq history 0 10)))

             (when (= (length history) 10)
               (fill counts 0)
               (dolist (h history)
                 (when (and (>= h 1) (<= h 6)) (incf (aref counts h))))

               (dotimes (i 6)
                 (let ((target-id (+ i 1)))
                   (when (>= (aref counts target-id) 7)
                     (setq detected-id target-id)
                     (setq looping nil)
                     (return))))
               ))))
       )
     ;; 確定したら推論モードOFF（通信量削減のため）
     (send self :send-classify-cmd "s")
     detected-id))

  (:execute-grasp (id &optional (mode :grasped))
   (unless id (return-from :execute-grasp nil))
   (format t ">> Executing Class [~A] in mode [~A]...~%" id mode)
   ;; (ros::duration-sleep 5.0)
   (case id
     (1 (if (eq mode :grasping)
            (send self :five-finger-hook-grasping)
          (send self :five-finger-hook-grasped)))
     (2 (send self :interlocked-fingers))
     (3 (if (eq mode :grasping)
            (send self :finger-hold-grasping)
            (send self :finger-hold-grasped)))
     (4 (send self :mutual-palm-grasp))
     (5 (if (eq mode :grasping)
            (send self :finger-cradle-grasp-grasping)
            (send self :finger-cradle-grasp-grasped)))
     (6 (if (eq mode :grasping)
            (send self :four-finger-hook-grasping)
            (send self :four-finger-hook-grasped)))
     (t (warn "Unknown ID: ~A~%" id)))
   (format t ">> Grasp Complete.~%"))

  ;; --- キー実行 ---
  (:auto-sequence ()
   (format t "=== Auto Sequence Start ===~%")

   ;; 1. 認識 & 2. 把持
   (let ((id (send self :detect-hand-class)))
     (when id
       (send self :execute-grasp id :grasped)
       (ros::duration-sleep 3.0)
       ;; 3. 監視 (Monitoring)
       (format t ">> [Step 3] Locking & Monitoring...~%")
       (send self :send-classify-cmd "m") ;; 監視モードON & ロック
       (ros::duration-sleep 0.5)

       (let ((monitoring t)
             (threshold 0.7))
         (while (and (ros::ok) monitoring)
           (let ((msg (one-shot-subscribe "classification_score" std_msgs::Float32)))
             (when msg
               (let ((score (send msg :data)))
                 (format t "\r Score: ~5,3F " score)
                 (when (< score threshold)
                   (format t "~%>> Release Triggered!~%")
                   (setq monitoring nil)))))

           (when (select-stream (list *standard-input*) 0.001)
             (let ((c (read-char *standard-input*)))
               (when (or (eq c #\q) (eq c #\c))
                 (format t "~%>> [User] Cancel detected. Forcing release...~%")
                 (setq monitoring nil))))
           (unix:usleep 50000)))

       ;; 4. 離脱
       (format t ">> [Step 4] Releasing...~%")
       (send self :hand-open)
       (send self :send-classify-cmd "m") ;; 監視モードOFF
       (format t "=== Sequence Finished ===~%")
       )))

  (:recognize-and-act ()
   (format t "=== Manual Recognition Start ===~%")
   (let ((id (send self :detect-hand-class)))
     (when id
       (send self :execute-grasp id :grasped)
       (format t ">> Done. (No monitoring)~%"))))

  ;; --- 各モーション定義 ---
  (:hand-open ()
   (format t ">> Executing: Hand Open~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 2.0)
   )

  (:five-finger-hook-grasped ()
   (format t ">> Executing: Five Finger Hook~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector 10.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector 10.0 -40.0 40.0 40.0 -40.0 35.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector 10.0 -35.0 35.0 35.0 -35.0 35.0 1.5))
   (ros::duration-sleep 2.0)
   ;; (send self :publish-angle (float-vector 10.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   ;; (ros::duration-sleep 1.0)
   ;; (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   ;; (ros::duration-sleep 1.0)
   )

  (:five-finger-hook-grasping ()
   (format t ">> Executing: Five Finger Hook grasping~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -50.0 -40.0 40.0 40.0 -40.0 35.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -50.0 -35.0 35.0 35.0 -35.0 35.0 1.5))
   (ros::duration-sleep 2.0)
   )

  (:interlocked-fingers ()
   (format t ">> Executing: Interlocked Fingers~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 -0.2))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -20.0 -40.0 40.0 40.0 -40.0 35.0 -0.2))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -20.0 -35.0 35.0 35.0 -35.0 35.0 -0.2))
   (ros::duration-sleep 2.0)
   ;; (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 -0.2))
   ;; (ros::duration-sleep 1.0)
   ;; (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   ;; (ros::duration-sleep 0.5)
   )

  (:finger-hold-grasped ()
   (format t ">> Executing: Finger Hold~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector 10.0 60.0 -60.0 10.0 60.0 -60.0 0.2))
   (ros::duration-sleep 3.5)
   ;; (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   ;; (ros::duration-sleep 1.0)
   )

  (:finger-hold-grasping ()
   (format t ">> Executing: Finger Hold Grasped~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector -20.0 -50.0 50.0 50.0 -50.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -20.0 -45.0 45.0 45.0 -45.0 -60.0 1.5))
   (ros::duration-sleep 2.0)
                         )

  (:mutual-palm-grasp ()
   (format t ">> Executing: Mutual Palm Grasp~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector 50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector 50.0 -40.0 40.0 40.0 -40.0 35.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector 50.0 -35.0 35.0 35.0 -35.0 35.0 1.5))
   (ros::duration-sleep 2.0)
   ;; (send self :publish-angle (float-vector 50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   ;; (ros::duration-sleep 1.0)
   ;; (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   ;; (ros::duration-sleep 1.0)
   )

  (:finger-cradle-grasp-grasped ()
   (format t ">> Executing: Finger Cradle Grasp~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector -50.0 -10.0 10.0 10.0 -10.0 -60.0 1.5))
   (ros::duration-sleep 4.0)
   ;; (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   ;; (ros::duration-sleep 1.0)
   )

  (:finger-cradle-grasp-grasping ()
   (format t ">> Executing: Finger Cradle Grasp grasping~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector 20.0 -10.0 10.0 10.0 -10.0 -60.0 0.2))
   (ros::duration-sleep 3.5)
   )

  (:four-finger-hook-grasped ()
   (format t ">> Executing: Four Finger Hook~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector 10.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector 10.0 -25.0 25.0 25.0 -25.0 25.0 1.5))
   (ros::duration-sleep 4.0)
   ;; (send self :publish-angle (float-vector 10.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   ;; (ros::duration-sleep 1.0)
   ;; (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   ;; (ros::duration-sleep 1.0)
   )

  (:four-finger-hook-grasping ()
   (format t ">> Executing: Four Finger Hook grasping~%")
   (send self :publish-angle (float-vector -50.0 60.0 -60.0 -60.0 60.0 -60.0 1.5))
   (ros::duration-sleep 0.1)
   (send self :publish-angle (float-vector -20.0 -40.0 40.0 40.0 -40.0 -60.0 1.5))
   (ros::duration-sleep 1.0)
   (send self :publish-angle (float-vector -20.0 -35.0 35.0 35.0 -35.0 -60.0 1.5))
   (ros::duration-sleep 2.0)
   )
  )

;; --- 2. メインループ ---

(defun main-loop ()
  (format t "
=== Hand Motion Controller ===
 [1] Five Finger Hook
 [2] Interlocked Fingers
 [3] Finger Hold
 [4] Mutual Palm Grasp
 [5] Finger Cradle Grasp
 [6] Four Finger Hook
 [o] Hand Open
 [g] Switch Grasping/Grasped
 [r] Start Recognition Mode (Wait for result)
 [a]  : Auto Sequence (Recognize -> Grasp -> Monitor -> Release)
 [q] Quit
------------------------------
")

  (let ((hand (instance hand-holding :init))
        (running t)
        (key nil)
        (grasp-mode :grasped))

    (while (and (ros::ok) running)
      (ros::spin-once)

      ;; キー入力チェック
      (when (select-stream (list *standard-input*) 0.001)
        (setq key (read-char *standard-input*))

        (case key
          ((#\1 #\2 #\3 #\4 #\5 #\6)
           ;; 文字コードから数値を変換 (#\1 -> 1)
           (let ((id (- key 48)))
             (send hand :execute-grasp id grasp-mode)))
          (#\g
           ;; モード切替
           (setq grasp-mode (if (eq grasp-mode :grasping) :grasped :grasping))
           (format t "=== Grasp-Mode: ~A ===~%" grasp-mode))
          (#\o (send hand :hand-open))
          (#\r (send hand :recognize-and-act)) ;; 認識モード呼び出し
          (#\a (send hand :auto-sequence))
          (#\q
           (format t "Exiting...~%")
           (setq running nil))
          (#\newline )
          (t (format t "Unknown key: ~C~%" key))
          )

        (unless (member key '(#\newline))
           (format t "Ready. [1-6/r/q] > ~%"))
        )

      (unix:usleep 10000)
      )
    ))

(main-loop)
