// SPDX-FileCopyrightText: Copyright (C) 2023-2025 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import { RawImage } from "@foxglove/schemas";

import * as Comlink from "@lichtblick/comlink";
import { ComlinkWrap } from "@lichtblick/den/worker";

import type { RawImageOptions } from "./decodeImage";
import { Image as RosImage } from "../../ros";

/**
 * Provides a worker that can process RawImages on a background thread.
 */

type WorkerService = (typeof import("./WorkerImageDecoder.worker"))["service"];

export class WorkerImageDecoder {
  #remote: Comlink.Remote<WorkerService>;
  #dispose: () => void;

  public constructor() {
    const { remote, dispose } = ComlinkWrap<WorkerService>(
      new Worker(
        // foxglove-depcheck-used: babel-plugin-transform-import-meta
        new URL("./WorkerImageDecoder.worker", import.meta.url),
        { type: "module" },        // 👈 IMPORTANT: tell the browser this is a module worker
      ),
    );
    this.#remote = remote;
    this.#dispose = dispose;
  }

  public async decode(
    image: RosImage | RawImage,
    options: Partial<RawImageOptions>,
  ): Promise<ImageData> {
    return await this.#remote.decode(image, options);
  }

  public terminate(): void {
    this.#dispose();
  }
}
