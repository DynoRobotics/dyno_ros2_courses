// Layout persistence service using localStorage
export interface ActionManagerPersistentState {
    selectedNamespace?: string;
    selectedAction?: string;
    goalValues?: Record<string, any>;
    lastUpdated?: Date;
}

export interface ActionManagerConfig {
    id: string;
    title: string;
    defaultNamespace?: string;
    allowedActions?: string[];
    collapsed?: boolean;
    persistentState?: ActionManagerPersistentState;
}

export interface FlowLayoutConfig {
    columnsPerRow: number; // 1, 2, 3, or 4 panels per row
    panelSpacing: number; // Gap between panels in pixels
    responsive: boolean; // Whether to adapt to screen size
    minPanelWidth: number; // Minimum width for panels
}

export interface LayoutConfig {
    id: string;
    name: string;
    description?: string;
    createdAt: Date;
    lastModified: Date;
    panelOrder: string[]; // Array of panel IDs in display order
    panels: Record<string, ActionManagerConfig>; // Panel configs by ID
    layoutConfig: FlowLayoutConfig;
}

class LayoutStorageService {
    private readonly STORAGE_KEY = 'ros2-web-ui-layouts';
    private readonly ACTIVE_LAYOUT_KEY = 'ros2-web-ui-active-layout';

    // Generate UUID for unique IDs
    private generateUUID(): string {
        return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, function (c) {
            const r = Math.random() * 16 | 0;
            const v = c === 'x' ? r : (r & 0x3 | 0x8);
            return v.toString(16);
        });
    }

    // Get all layouts from localStorage
    getAllLayouts(): LayoutConfig[] {
        try {
            const stored = localStorage.getItem(this.STORAGE_KEY);
            if (!stored) return [];

            const layouts = JSON.parse(stored);
            // Convert date strings back to Date objects
            return layouts.map((layout: any) => ({
                ...layout,
                createdAt: new Date(layout.createdAt),
                lastModified: new Date(layout.lastModified)
            }));
        } catch (error) {
            console.error('Error loading layouts:', error);
            return [];
        }
    }

    // Get a specific layout by ID
    getLayout(id: string): LayoutConfig | null {
        const layouts = this.getAllLayouts();
        const layout = layouts.find(layout => layout.id === id) || null;

        // Ensure backward compatibility for layouts without panelOrder or layoutConfig
        if (layout) {
            if (!layout.panelOrder) {
                layout.panelOrder = [];
            }
            if (!layout.layoutConfig) {
                layout.layoutConfig = {
                    columnsPerRow: 2,
                    panelSpacing: 16,
                    responsive: true,
                    minPanelWidth: 400
                };
            }
        }

        return layout;
    }

    // Save a layout
    saveLayout(layout: LayoutConfig): void {
        try {
            const layouts = this.getAllLayouts();
            const index = layouts.findIndex(l => l.id === layout.id);

            const updatedLayout = {
                ...layout,
                lastModified: new Date()
            };

            if (index >= 0) {
                layouts[index] = updatedLayout;
            } else {
                layouts.push(updatedLayout);
            }

            localStorage.setItem(this.STORAGE_KEY, JSON.stringify(layouts));
        } catch (error) {
            console.error('Error saving layout:', error);
            throw new Error('Failed to save layout');
        }
    }

    // Delete a layout
    deleteLayout(id: string): void {
        try {
            const layouts = this.getAllLayouts();
            const filteredLayouts = layouts.filter(layout => layout.id !== id);
            localStorage.setItem(this.STORAGE_KEY, JSON.stringify(filteredLayouts));

            // If this was the active layout, clear it
            if (this.getActiveLayoutId() === id) {
                this.setActiveLayout(null);
            }
        } catch (error) {
            console.error('Error deleting layout:', error);
            throw new Error('Failed to delete layout');
        }
    }

    // Create a new layout
    createLayout(name: string, description?: string): LayoutConfig {
        const layout: LayoutConfig = {
            id: this.generateUUID(),
            name,
            description,
            createdAt: new Date(),
            lastModified: new Date(),
            panelOrder: [],
            panels: {},
            layoutConfig: {
                columnsPerRow: 2,
                panelSpacing: 16,
                responsive: true,
                minPanelWidth: 400
            }
        };

        this.saveLayout(layout);
        return layout;
    }

    // Duplicate a layout
    duplicateLayout(id: string, newName?: string): LayoutConfig | null {
        const originalLayout = this.getLayout(id);
        if (!originalLayout) return null;

        // Create new panel IDs and update the mapping
        const newPanelIds: Record<string, string> = {};
        const newPanels: Record<string, ActionManagerConfig> = {};

        Object.entries(originalLayout.panels).forEach(([oldId, panel]) => {
            const newId = this.generateUUID();
            newPanelIds[oldId] = newId;
            newPanels[newId] = {
                ...panel,
                id: newId
            };
        });

        const duplicatedLayout: LayoutConfig = {
            ...originalLayout,
            id: this.generateUUID(),
            name: newName || `${originalLayout.name} (Copy)`,
            createdAt: new Date(),
            lastModified: new Date(),
            panelOrder: originalLayout.panelOrder.map(oldId => newPanelIds[oldId]),
            panels: newPanels
        };

        this.saveLayout(duplicatedLayout);
        return duplicatedLayout;
    }

    // Export layout as JSON string
    exportLayout(id: string): string | null {
        const layout = this.getLayout(id);
        if (!layout) return null;

        return JSON.stringify(layout, null, 2);
    }

    // Import layout from JSON string
    importLayout(jsonString: string, newName?: string): LayoutConfig {
        try {
            const layout = JSON.parse(jsonString) as LayoutConfig;

            // Generate new IDs to avoid conflicts
            const newPanelIds: Record<string, string> = {};
            const newPanels: Record<string, ActionManagerConfig> = {};

            Object.entries(layout.panels).forEach(([oldId, panel]) => {
                const newId = this.generateUUID();
                newPanelIds[oldId] = newId;
                newPanels[newId] = {
                    ...panel,
                    id: newId
                };
            });

            const importedLayout: LayoutConfig = {
                ...layout,
                id: this.generateUUID(),
                name: newName || `${layout.name} (Imported)`,
                createdAt: new Date(),
                lastModified: new Date(),
                panelOrder: layout.panelOrder.map(oldId => newPanelIds[oldId]),
                panels: newPanels
            };

            this.saveLayout(importedLayout);
            return importedLayout;
        } catch (error) {
            console.error('Error importing layout:', error);
            throw new Error('Invalid layout file');
        }
    }

    // Active layout management
    getActiveLayoutId(): string | null {
        return localStorage.getItem(this.ACTIVE_LAYOUT_KEY);
    }

    setActiveLayout(id: string | null): void {
        if (id) {
            localStorage.setItem(this.ACTIVE_LAYOUT_KEY, id);
        } else {
            localStorage.removeItem(this.ACTIVE_LAYOUT_KEY);
        }
    }

    getActiveLayout(): LayoutConfig | null {
        const activeId = this.getActiveLayoutId();
        return activeId ? this.getLayout(activeId) : null;
    }

    // Create default layout if none exists
    getOrCreateDefaultLayout(): LayoutConfig {
        const layouts = this.getAllLayouts();

        if (layouts.length === 0) {
            // Create a default layout with one ActionManager panel
            const defaultLayout = this.createLayout(
                'Default Layout',
                'Default single panel layout'
            );

            // Add a default panel
            const panelId = this.generateUUID();
            defaultLayout.panels = {
                [panelId]: {
                    id: panelId,
                    title: 'Action Manager',
                    collapsed: false
                }
            };
            defaultLayout.panelOrder = [panelId];

            this.saveLayout(defaultLayout);
            this.setActiveLayout(defaultLayout.id);
            return defaultLayout;
        }

        // Return active layout or first available layout
        const activeLayout = this.getActiveLayout();
        if (activeLayout) return activeLayout;

        const firstLayout = layouts[0];
        this.setActiveLayout(firstLayout.id);
        return firstLayout;
    }

    // Layout templates
    createTemplateLayout(templateName: string): LayoutConfig {
        const createPanelData = (title: string, defaultNamespace?: string, allowedActions?: string[]) => {
            const id = this.generateUUID();
            return {
                id,
                panel: {
                    id,
                    title,
                    defaultNamespace,
                    allowedActions,
                    collapsed: false
                }
            };
        };

        const templates: Record<string, { name: string; description: string; panelData: ReturnType<typeof createPanelData>[] }> = {
            'multi-robot': {
                name: 'Multi-Robot Control',
                description: 'Layout for controlling multiple robots',
                panelData: [
                    createPanelData('Robot 1', 'turtle1'),
                    createPanelData('Robot 2', 'turtle2'),
                    createPanelData('Robot 3', 'turtle3')
                ]
            },
            'testing': {
                name: 'Action Testing',
                description: 'Layout for testing different actions',
                panelData: [
                    createPanelData('Move Actions', undefined, ['move']),
                    createPanelData('Rotate Actions', undefined, ['rotate']),
                    createPanelData('All Actions')
                ]
            },
            'monitoring': {
                name: 'Monitoring Dashboard',
                description: 'Layout for monitoring multiple systems',
                panelData: [
                    createPanelData('System 1'),
                    createPanelData('System 2'),
                    createPanelData('System 3'),
                    createPanelData('System 4')
                ]
            }
        };

        const template = templates[templateName];
        if (!template) {
            throw new Error(`Template '${templateName}' not found`);
        }

        const panels: Record<string, ActionManagerConfig> = {};
        const panelOrder: string[] = [];

        template.panelData.forEach(({ id, panel }) => {
            panels[id] = panel;
            panelOrder.push(id);
        });

        const layout: LayoutConfig = {
            id: this.generateUUID(),
            name: template.name,
            description: template.description,
            createdAt: new Date(),
            lastModified: new Date(),
            panelOrder,
            panels,
            layoutConfig: {
                columnsPerRow: 3,
                panelSpacing: 16,
                responsive: true,
                minPanelWidth: 350
            }
        };

        this.saveLayout(layout);
        return layout;
    }

    // Helper methods for the new flow-based layout
    addPanel(layoutId: string, panel: Omit<ActionManagerConfig, 'id'>): ActionManagerConfig | null {
        const layout = this.getLayout(layoutId);
        if (!layout) return null;

        const newPanel: ActionManagerConfig = {
            ...panel,
            id: this.generateUUID()
        };

        // Ensure panelOrder exists (for backward compatibility)
        if (!layout.panelOrder) {
            layout.panelOrder = [];
        }

        layout.panels[newPanel.id] = newPanel;
        layout.panelOrder.push(newPanel.id);
        layout.lastModified = new Date();

        this.saveLayout(layout);
        return newPanel;
    }

    removePanel(layoutId: string, panelId: string): boolean {
        const layout = this.getLayout(layoutId);
        if (!layout || !layout.panels[panelId]) return false;

        delete layout.panels[panelId];

        // Ensure panelOrder exists (for backward compatibility)
        if (!layout.panelOrder) {
            layout.panelOrder = [];
        } else {
            layout.panelOrder = layout.panelOrder.filter(id => id !== panelId);
        }

        layout.lastModified = new Date();

        this.saveLayout(layout);
        return true;
    }

    reorderPanels(layoutId: string, newOrder: string[]): boolean {
        const layout = this.getLayout(layoutId);
        if (!layout) return false;

        // Validate that all panel IDs exist
        const validIds = newOrder.filter(id => layout.panels[id]);
        if (validIds.length !== Object.keys(layout.panels).length) return false;

        layout.panelOrder = validIds;
        layout.lastModified = new Date();

        this.saveLayout(layout);
        return true;
    }

    updateLayoutConfig(layoutId: string, newConfig: Partial<FlowLayoutConfig>): boolean {
        const layout = this.getLayout(layoutId);
        if (!layout) return false;

        layout.layoutConfig = { ...layout.layoutConfig, ...newConfig };
        layout.lastModified = new Date();

        this.saveLayout(layout);
        return true;
    }

    // Persistent state management for ActionManager panels
    updatePanelState(layoutId: string, panelId: string, state: Partial<ActionManagerPersistentState>): boolean {
        const layout = this.getLayout(layoutId);
        if (!layout || !layout.panels[panelId]) return false;

        const panel = layout.panels[panelId];
        panel.persistentState = {
            ...panel.persistentState,
            ...state,
            lastUpdated: new Date()
        };

        layout.lastModified = new Date();
        this.saveLayout(layout);
        return true;
    }

    getPanelState(layoutId: string, panelId: string): ActionManagerPersistentState | null {
        const layout = this.getLayout(layoutId);
        if (!layout || !layout.panels[panelId]) return null;

        return layout.panels[panelId].persistentState || null;
    }

    clearPanelState(layoutId: string, panelId: string): boolean {
        const layout = this.getLayout(layoutId);
        if (!layout || !layout.panels[panelId]) return false;

        delete layout.panels[panelId].persistentState;
        layout.lastModified = new Date();
        this.saveLayout(layout);
        return true;
    }

    // Utility method to validate if persistent state is still valid
    validatePersistentState(state: ActionManagerPersistentState, availableActions: string[], availableNamespaces: string[]): ActionManagerPersistentState {
        const validatedState: ActionManagerPersistentState = { ...state };

        // Validate namespace
        if (validatedState.selectedNamespace && !availableNamespaces.includes(validatedState.selectedNamespace)) {
            delete validatedState.selectedNamespace;
            delete validatedState.selectedAction; // Clear action if namespace is invalid
            validatedState.goalValues = {}; // Clear goal values if action is invalid
        }

        // Validate action
        if (validatedState.selectedAction && !availableActions.includes(validatedState.selectedAction)) {
            delete validatedState.selectedAction;
            validatedState.goalValues = {}; // Clear goal values if action is invalid
        }

        return validatedState;
    }
}

// Export singleton instance
export const layoutService = new LayoutStorageService();
