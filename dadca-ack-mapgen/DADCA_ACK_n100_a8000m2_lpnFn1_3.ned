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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=138.10,42.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=81.77,7.36");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=9.31,31.25");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=15.07,93.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=183.75,87.63");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=175.57,154.62");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=213.86,13.20");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=68.82,182.59");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=351.38,28.95");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=301.01,95.79");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=335.54,203.82");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=404.57,214.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=488.50,187.67");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=465.82,249.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=514.35,118.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=420.66,73.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=461.26,18.77");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=545.97,60.59");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=606.94,102.04");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=550.83,230.51");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=592.20,296.57");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=424.75,321.01");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=344.96,286.82");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=295.46,347.81");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=262.52,285.22");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=177.29,293.44");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=160.45,382.25");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=100.82,391.30");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=225.87,211.92");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=428.53,427.26");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=489.66,411.31");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=480.61,346.95");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=686.72,96.21");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=729.39,168.03");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=669.85,7.40");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=606.99,11.55");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=635.54,198.60");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=715.82,252.46");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=807.84,264.54");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=850.15,213.44");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=932.77,181.59");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=774.49,211.37");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=731.36,330.93");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=655.55,385.87");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=565.98,390.03");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=614.67,431.81");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=561.39,541.37");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=556.31,630.10");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=665.12,547.13");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=678.67,473.93");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=504.95,580.73");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=441.01,589.70");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=466.52,658.01");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=440.65,503.43");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=663.83,685.06");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=585.90,726.76");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=638.01,776.65");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=521.16,708.42");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=503.93,498.71");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=637.57,605.14");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=701.36,608.12");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=601.53,490.42");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=369.95,483.78");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=312.33,514.52");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=328.54,604.15");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=307.41,446.75");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=363.56,404.99");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=578.23,155.73");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=754.14,480.69");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=788.32,540.18");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=836.51,639.91");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=911.22,683.90");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=950.68,745.89");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=890.68,804.05");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=888.39,894.65");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=872.57,987.05");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=799.69,978.92");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=706.16,957.07");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=763.89,904.04");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=843.58,842.95");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=949.80,902.17");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=954.73,840.40");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1072.55,883.98");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1121.33,941.65");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1204.24,981.28");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1139.37,1013.01");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1236.72,1070.36");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1300.42,1013.98");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1334.55,963.52");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=1422.64,921.53");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1383.82,835.21");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1314.12,844.18");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1427.64,776.41");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1451.84,865.78");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1235.27,913.13");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1309.32,1098.96");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1399.56,1007.18");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1245.80,1150.44");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1293.72,1195.51");
    }

}

