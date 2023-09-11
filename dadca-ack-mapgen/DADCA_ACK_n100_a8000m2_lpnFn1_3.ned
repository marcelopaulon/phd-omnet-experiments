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

network DADCA_ACK_n100_a8000m2_lpnFn1_3
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
        @display("i=misc/sensor,p=134.89,41.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=100.97,217.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=30.84,230.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=153.11,178.23");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=199.16,103.08");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=271.14,50.55");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=299.77,139.06");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=371.61,75.05");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=288.82,278.90");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=240.57,216.61");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=206.52,270.55");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=144.68,262.03");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=47.10,331.30");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=14.51,394.67");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=107.29,324.12");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=457.61,65.64");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=504.00,132.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=565.41,204.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=458.03,196.46");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=373.77,246.80");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=161.48,362.74");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=158.67,439.62");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=233.57,378.12");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=97.49,447.28");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=125.19,513.51");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=160.50,581.04");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=221.47,522.01");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=49.87,494.60");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=252.29,446.36");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=91.71,382.10");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=26.45,609.29");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=92.35,571.02");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=345.62,550.37");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=232.06,583.79");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=293.03,676.52");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=381.85,669.05");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=420.62,606.86");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=243.83,745.05");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=231.66,683.90");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=177.86,638.76");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=88.80,692.41");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=136.26,774.77");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=383.12,187.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=377.16,0.83");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=636.42,112.04");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=561.11,100.29");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=705.60,64.30");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=626.81,7.19");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=646.73,189.04");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=533.65,259.10");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=546.59,350.59");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=613.19,287.14");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=691.01,251.98");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=608.53,372.29");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=680.08,326.57");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=773.67,298.26");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=853.89,306.44");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=911.24,333.42");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=952.25,377.51");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=990.44,328.46");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1072.16,371.73");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1075.94,304.62");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1107.23,219.63");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1023.49,209.23");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=949.59,184.02");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1078.37,113.49");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1183.88,312.37");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=970.40,259.29");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=993.94,483.60");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1038.07,543.75");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1089.82,581.50");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1021.05,641.42");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=933.32,635.32");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=942.56,550.32");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=894.71,488.22");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=816.22,420.87");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=829.23,494.53");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=816.87,561.38");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=816.32,653.01");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=831.14,727.30");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=886.84,700.53");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=696.58,399.72");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=510.90,409.14");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=564.08,439.10");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=522.34,484.14");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=604.96,535.24");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=588.33,610.59");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=441.58,409.38");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=365.40,408.62");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=288.65,350.75");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=409.44,348.23");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=475.95,336.61");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=756.94,358.56");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=892.48,422.06");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1130.72,344.51");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=991.39,137.59");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1031.60,55.99");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1122.88,67.63");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1212.45,42.79");
    }

}

