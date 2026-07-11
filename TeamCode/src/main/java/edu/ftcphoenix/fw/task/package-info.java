/**
 * Cooperative, single-use robot behavior driven by one loop heartbeat.
 *
 * <p>Task instances follow one lifecycle: start once, update while active, then complete or cancel.
 * Framework Tasks reject update-before-start, ignore cancellation before start, become terminal on
 * active cancellation, and ignore terminal or repeated cancellation. {@link
 * edu.ftcphoenix.fw.task.Tasks#noop()} is already successfully complete when created and is the
 * intentional direct-update exception. {@link
 * edu.ftcphoenix.fw.task.TaskRunner} owns sequential execution and fails closed when task lifecycle
 * code throws; {@link edu.ftcphoenix.fw.task.OutputTaskRunner} adds source-driven scalar output and
 * returns to its idle output after abort or failure.</p>
 */
package edu.ftcphoenix.fw.task;
