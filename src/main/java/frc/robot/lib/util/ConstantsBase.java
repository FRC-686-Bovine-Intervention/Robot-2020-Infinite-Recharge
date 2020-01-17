package frc.robot.lib.util;

import java.lang.reflect.Field;
import java.util.*;

/**
 * ConstantsBase
 * 
 * Base class for storing robot constants. Anything stored as a public static
 * field will be reflected and be able to set externally
 */
public abstract class ConstantsBase {
    HashMap<String, Boolean> modifiedKeys = new HashMap<String, Boolean>();

    public static class Constant {
        public String name;
        public Class<?> type;
        public Object value;

        public Constant(String name, Class<?> type, Object value) {
            this.name = name;
            this.type = type;
            this.value = value;
        }

        @Override
        public boolean equals(Object o) {
            String itsName = ((Constant) o).name;
            Class<?> itsType = ((Constant) o).type;
            Object itsValue = ((Constant) o).value;
            return o instanceof Constant && this.name.equals(itsName) && this.type.equals(itsType)
                    && this.value.equals(itsValue);
        }
    }

    public boolean setConstant(String name, Double value) {
        return setConstantRaw(name, value);
    }

    public boolean setConstant(String name, Integer value) {
        return setConstantRaw(name, value);
    }

    public boolean setConstant(String name, String value) {
        return setConstantRaw(name, value);
    }

    private boolean setConstantRaw(String name, Object value) {
        boolean success = false;
        Field[] declaredFields = this.getClass().getDeclaredFields();
        for (Field field : declaredFields) {
            if (java.lang.reflect.Modifier.isStatic(field.getModifiers()) && field.getName().equals(name)) {
                try {
                    Object current = field.get(this);
                    field.set(this, value);
                    success = true;
                    if (!value.equals(current)) {
                        modifiedKeys.put(name, true);
                    }
                } catch (IllegalArgumentException | IllegalAccessException e) {
                    System.out.println("Could not set field: " + name);
                }
            }
        }
        return success;
    }

    public Object getValueForConstant(String name) throws Exception {
        Field[] declaredFields = this.getClass().getDeclaredFields();
        for (Field field : declaredFields) {
            if (java.lang.reflect.Modifier.isStatic(field.getModifiers()) && field.getName().equals(name)) {
                try {
                    return field.get(this);
                } catch (IllegalArgumentException | IllegalAccessException e) {
                    throw new Exception("Constant not found");
                }
            }
        }
        throw new Exception("Constant not found");
    }

    public Constant getConstant(String name) {
        Field[] declaredFields = this.getClass().getDeclaredFields();
        for (Field field : declaredFields) {
            if (java.lang.reflect.Modifier.isStatic(field.getModifiers()) && field.getName().equals(name)) {
                try {
                    return new Constant(field.getName(), field.getType(), field.get(this));
                } catch (IllegalArgumentException | IllegalAccessException e) {
                    e.printStackTrace();
                }
            }
        }
        return new Constant("", Object.class, 0);
    }

    public Collection<Constant> getConstants() {
        List<Constant> constants = (List<Constant>) getAllConstants();
        int stop = constants.size() - 1;
        for (int i = 0; i < constants.size(); ++i) {
            Constant c = constants.get(i);
            if ("kEndEditableArea".equals(c.name)) {
                stop = i;
            }
        }
        return constants.subList(0, stop);
    }

    private Collection<Constant> getAllConstants() {
        Field[] declaredFields = this.getClass().getDeclaredFields();
        List<Constant> constants = new ArrayList<Constant>(declaredFields.length);
        for (Field field : declaredFields) {
            if (java.lang.reflect.Modifier.isStatic(field.getModifiers())) {
                Constant c;
                try {
                    c = new Constant(field.getName(), field.getType(), field.get(this));
                    constants.add(c);
                } catch (IllegalArgumentException | IllegalAccessException e) {
                    e.printStackTrace();
                }
            }
        }
        return constants;
    }

 }
