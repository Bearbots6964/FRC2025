package frc.robot.util

import edu.wpi.first.util.struct.Struct
import java.nio.ByteBuffer

class CommandQueueStruct: Struct<CommandQueue> {
    /**
     * Gets the Class object for the stored value.
     *
     * @return Class
     */
    override fun getTypeClass(): Class<CommandQueue> {
        return CommandQueue::class.java
    }

    /**
     * Gets the type name (e.g. for schemas of other structs). This should be globally unique among
     * structs.
     *
     * @return type name
     */
    override fun getTypeName(): String {
        return "CommandQueue"
    }

    /**
     * Gets the serialized size (in bytes). This should always be a constant.
     *
     * @return serialized size
     */
    override fun getSize(): Int {
        return 8240
    }

    /**
     * Gets the schema.
     *
     * @return schema
     */
    override fun getSchema(): String {
        return "uint8 size;" +
                "char name[32];" +
                "uint8 state;" +
                "char queue[8192];"
    }

    /**
     * Deserializes an object from a raw struct serialized ByteBuffer starting at the current
     * position. Will increment the ByteBuffer position by getStructSize() bytes. Will not otherwise
     * modify the ByteBuffer (e.g. byte order will not be changed).
     *
     * @param bb ByteBuffer
     * @return New object
     */
    override fun unpack(bb: ByteBuffer?): CommandQueue {
        val queue = CommandQueue()
        val size = bb!!.get().toInt()
        val name = ByteArray(32)
        bb.get(name)
        val nameStringified = String(name)
        val state = ByteArray(8)
        bb.get(state)
        // queue is sliced into 256-byte command names
        val queueList = ByteArray(8192)
        bb.get(queueList)
        val queueStringified = Array<String>(size) { "" }
        for (i in 0..size) {
            queueStringified[i] = String(queueList.sliceArray(i * 256 until (i + 1) * 256))
        }
        queue.state = (CommandQueue.CommandQueueState.valueOf(String(state)))
        queue.dumpFromStruct(queueStringified)
        queue.name = (nameStringified)
        return queue
    }

    /**
     * Puts object contents to a ByteBuffer starting at the current position. Will increment the
     * ByteBuffer position by getStructSize() bytes. Will not otherwise modify the ByteBuffer (e.g.
     * byte order will not be changed).
     *
     * @param bb ByteBuffer
     * @param value object to serialize
     */
    override fun pack(bb: ByteBuffer?, value: CommandQueue?) {
        bb!!.put(value!!.queue.size.toByte())
        val name = value.name.toByteArray()
        val paddedName = ByteArray(32, { 0 })
        for (i in name.indices) {
            paddedName[i] = name[i]
        }
        bb.put(paddedName.sliceArray(0..31))
        val state = value.state.ordinal.toByte()
        bb.put(state)
        val queueList = ByteArray(8192, { 0 })
        for (i in 0..<value.queue.size) {
            val command: ByteArray = value.queue[i].invoke().name.toByteArray().copyOf(256)
            for (j in command.indices) {
                queueList[i * 256 + j] = command[j]
            }
        }
        bb.put(queueList.sliceArray(0..8191))
    }

}
