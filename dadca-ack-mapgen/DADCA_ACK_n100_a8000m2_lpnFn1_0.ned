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

network DADCA_ACK_n100_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=179.86,126.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=84.68,158.63");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=51.61,246.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=14.81,315.75");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=3.31,385.43");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=20.75,480.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=18.04,552.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=112.14,558.76");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=192.21,559.59");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=118.49,673.00");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=95.06,395.38");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=84.87,301.05");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=142.61,347.06");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=166.66,281.14");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=224.93,296.00");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=4.67,206.30");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=174.70,196.14");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=254.60,104.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=306.82,70.44");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=289.16,0.91");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=241.61,44.07");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=301.63,187.33");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=360.89,122.62");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=236.27,188.72");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=399.59,59.69");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=163.57,60.13");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=4.83,57.86");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=59.39,16.40");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=278.82,246.90");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=349.52,314.54");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=351.59,235.95");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=406.32,187.86");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=275.30,333.07");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=396.31,387.60");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=408.88,462.85");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=334.13,501.12");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=272.73,440.21");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=258.58,516.83");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=166.21,436.56");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=474.64,187.30");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=421.34,118.55");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=540.84,138.32");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=550.56,235.08");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=493.35,286.01");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=553.09,321.88");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=628.44,234.04");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=669.57,187.62");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=603.64,177.48");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=547.36,46.69");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=460.13,13.95");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=493.71,91.96");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=418.56,261.70");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=445.05,350.46");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=316.26,390.17");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=635.71,9.35");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=599.04,78.40");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=745.86,86.93");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=685.66,129.44");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=698.31,32.22");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=832.33,59.51");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=839.58,153.07");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=742.55,176.54");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=901.38,161.46");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1012.09,174.98");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1107.64,196.90");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1078.99,140.31");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1090.25,46.38");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1183.73,49.10");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1008.17,10.30");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=940.37,64.09");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=993.95,112.50");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1074.45,286.09");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1101.14,347.81");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1134.57,427.87");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1185.93,304.71");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1031.84,432.05");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1070.09,483.30");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=1039.29,577.55");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1068.15,660.14");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1000.16,624.24");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=988.44,712.52");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1047.81,744.02");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1108.96,764.68");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1190.57,795.35");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1135.62,847.89");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1160.74,940.43");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1180.50,1011.33");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1241.08,1009.61");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1081.11,982.74");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1087.30,889.43");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1023.64,938.85");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1161.90,719.15");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1298.99,724.30");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1359.08,724.66");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1311.12,796.58");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1290.52,889.58");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1231.90,918.60");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1372.69,944.56");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1434.07,985.44");
    }

}

