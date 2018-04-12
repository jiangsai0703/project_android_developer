/*
 * Copyright (C) 2015 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.android.preload.actions;

import com.android.preload.DumpTableModel;
import com.android.preload.Main;

public class ComputeThresholdXAction extends ComputeThresholdAction {

    public ComputeThresholdXAction(String name, DumpTableModel dataTableModel,
            String blacklist) {
        super(name, dataTableModel, 1, blacklist);
    }

    @Override
    public void run() {
        String value = Main.getUI().showInputDialog("Threshold?");

        if (value != null) {
            try {
                threshold = Integer.parseInt(value);
                super.run();
            } catch (Exception exc) {
            }
        }
    }
}