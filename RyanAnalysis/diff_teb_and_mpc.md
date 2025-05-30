```
MPC 是利用機器人的移動模型來預測未來的軌跡，這個軌跡同時也要盡量滿足「約束條件」與「目標函數」，通常代表的就是未來的移動軌跡要貼和全局軌跡。接著會根據優化結果，取第一筆cmd來控制機器人，後續則不斷執行相同流程以滿足路徑追蹤的表現。
```

您的理解很好！TEB Local Planner 和 MPC (Model Predictive Control) 確實有一些相似之處，但也有重要的區別。讓我來比較這兩者：

### 相似點

1. **預測性質**：兩者都考慮未來的一段時間/路徑，而不僅僅是當前狀態
2. **優化過程**：兩者都通過求解優化問題來生成控制指令
3. **約束處理**：兩者都能處理各種約束（如避障、速度限制）
4. **循環執行**：兩者都在每個控制週期重新計算

### 關鍵區別

1. **優化變量**：
   - **MPC**：直接優化控制輸入序列（如加速度、轉向）
   - **TEB**：優化軌跡本身（位姿序列和時間差）

2. **模型依賴性**：
   - **MPC**：強烈依賴精確的機器人動力學模型進行狀態預測
   - **TEB**：相對不那麼依賴精確模型，更多關注軌跡的幾何和時間特性

3. **控制實現**：
   - **MPC**：優化控制序列後，通常只執行第一個控制指令
   - **TEB**：從優化後的軌跡中提取速度指令，可能會考慮更多點

4. **時間處理**：
   - **MPC**：通常使用固定的時間步長
   - **TEB**：時間差是優化變量的一部分，可以自適應變化

5. **計算方法**：
   - **MPC**：通常涉及線性化和二次規劃（QP）
   - **TEB**：通常使用非線性優化（如 g2o 框架）

### 總結

您的MPC描述基本準確。TEB可以看作是一種特殊形式的MPC，它更專注於直接優化軌跡而不是控制序列，並且將時間作為優化變量的一部分。TEB的一個主要優勢是能夠同時優化路徑的空間形狀和時間分配，使其特別適合考慮機器人動力學約束的本地導航。

兩者都是先進的路徑規劃方法，選擇哪一個通常取決於具體應用場景、可用的計算資源以及對模型精確度的要求。