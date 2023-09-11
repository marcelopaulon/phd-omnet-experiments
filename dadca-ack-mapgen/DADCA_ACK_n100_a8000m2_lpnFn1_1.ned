package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_a8000m2_lpnFn1_1
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=43.15,134.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=42.34,66.04");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=176.52,196.27");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=163.71,83.22");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=242.84,22.63");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=224.77,133.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=70.63,198.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=117.59,276.10");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=82.88,353.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=74.89,444.19");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=137.62,392.80");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=201.96,349.28");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=188.22,426.59");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=147.03,484.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=3.45,459.86");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=263.44,409.68");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=138.31,559.53");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=95.59,613.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=47.06,701.41");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=94.45,783.92");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=186.45,797.50");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=169.30,705.58");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=206.94,546.78");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=301.94,344.57");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=327.03,464.73");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=328.32,563.85");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=415.88,542.80");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=268.17,503.47");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=194.63,632.29");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=22.15,346.83");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=41.72,285.82");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=11.82,216.82");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=320.68,118.86");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=394.63,139.65");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=43.09,570.78");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=297.16,645.98");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=368.82,674.29");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=453.06,667.81");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=357.50,751.55");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=524.56,725.16");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=434.74,732.62");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=300.84,792.95");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=254.41,831.68");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=288.01,909.70");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=371.36,919.46");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=463.67,888.87");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=516.24,949.19");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=564.86,1004.71");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=478.16,1041.55");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=409.27,984.79");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=653.35,1021.46");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=710.31,982.67");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=771.97,977.44");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=794.26,1073.60");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=839.43,1145.37");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=740.37,1133.18");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=745.33,1218.27");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=645.16,1206.31");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=654.08,1115.09");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=865.79,1084.35");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=863.06,1213.51");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=730.49,1315.48");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=756.78,1402.75");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=774.64,1475.59");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=718.37,1549.36");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=803.60,1528.37");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=875.38,1481.62");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=832.66,1435.88");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=876.31,1382.93");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=948.39,1410.82");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=595.79,1319.46");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=602.20,1407.63");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=611.40,1468.54");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=544.81,1357.56");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=541.08,1259.48");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=495.37,1216.39");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=469.39,1374.67");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=491.91,1468.85");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=700.28,1451.25");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=676.56,1354.52");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=410.22,1304.78");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=326.88,1278.92");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=390.90,1235.73");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=648.49,1274.04");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=508.13,1139.70");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=431.47,1100.60");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=360.01,1048.27");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=249.60,1154.86");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=208.01,1237.96");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=267.05,1292.54");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=321.99,1341.75");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=247.90,1384.48");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=179.59,1349.63");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=150.14,1445.18");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=215.88,1449.99");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=221.31,1521.81");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=278.16,1545.05");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=363.37,1573.88");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=404.40,1649.53");
    }

}

