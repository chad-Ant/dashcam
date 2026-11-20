/** @file GenerationFiles.h - Portable primary/backup/temporary record persistence. */
#pragma once

namespace es3storage {
/** File names of one persisted record: the live file, its previous generation
 * and the scratch file a new generation is written and verified in. */
struct GenerationPaths {
    const char* primary;
    const char* backup;
    const char* temporary;
};

// Store supplies read(path, record)/write(path, record)/exists/remove/rename.
// Preserve a readable generation across short writes and failed renames; FAT
// power-loss atomicity is not assumed.
template<class Store, class Record>
bool loadGeneration(Store& store, const GenerationPaths& paths, Record& record) {
    return store.read(paths.primary, record) || store.read(paths.backup, record);
}

/** Writes and verifies the temporary file, keeps the old primary as the
 * backup, then promotes. same(a, b) compares two records. */
template<class Store, class Record, class Same>
bool saveGeneration(Store& store, const GenerationPaths& paths, const Record& record, Same same) {
    if (store.exists(paths.temporary) && !store.remove(paths.temporary)) return false;
    Record check{};
    if (!store.write(paths.temporary, record) || !store.read(paths.temporary, check) ||
        !same(check, record)) return false;
    if (store.exists(paths.primary)) {
        if (!store.exists(paths.backup) || store.read(paths.primary, check)) {
            if (store.exists(paths.backup) && !store.remove(paths.backup)) return false;
            if (!store.rename(paths.primary, paths.backup)) return false;
        } else {
            // A read failure may be transient. Never remove the primary unless
            // another readable generation is known to survive a failed promote.
            if (!store.read(paths.backup, check) || !store.remove(paths.primary)) return false;
        }
    }
    return store.rename(paths.temporary, paths.primary) &&
        store.read(paths.primary, check) && same(check, record);
}

template<class Store>
bool clearGeneration(Store& store, const GenerationPaths& paths) {
    // Remove backup first: a failed primary removal must leave the active file.
    const char* order[] = {paths.backup, paths.temporary, paths.primary};
    for (const char* path : order) {
        if (store.exists(path) && !store.remove(path)) return false;
    }
    return true;
}
}  // namespace es3storage
