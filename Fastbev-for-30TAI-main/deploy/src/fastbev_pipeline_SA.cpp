/*
 * FastBEV synchronous Group4/SA validation pipeline.
 *
 * The implementation reuses the stable synchronous pipeline helpers and
 * enables only the Group4 execution path in this translation unit. The plain
 * synchronous implementation remains a shared source, not a standalone target.
 * SA result rows include velocity fields:
 *   x y z w l h yaw vx vy class_id score
 */
#define FASTBEV_PIPELINE_SA 1
#include "fastbev_pipeline.cpp"
