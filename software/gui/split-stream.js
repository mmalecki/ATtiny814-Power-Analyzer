class SplitStream extends TransformStream {
  constructor(delimeter) {
    super({
      transform: (chunk, controller) => {
        let index;
        let rest = chunk;
        while ((index = rest.indexOf(this.delimeter)) !== -1) {
          for (const buf of this._buffer) controller.enqueue(buf)
          controller.enqueue(rest.slice(0, index));
          rest = rest.slice(index + 1);
          this._buffer = [];
        }

        if (rest.length > 0) {
          this._buffer.push(rest);
        }
      },
      flush: (controller) => {
        for (const buf of this._buffer) controller.enqueue(buf)
      }
    })

    if (typeof delimeter !== 'string' && delimeter.length !== 1)
      throw new TypeError('Delimeter is required and must be one character long')

    this.delimeter = delimeter
    this._buffer = []
  }
}

export { SplitStream }
