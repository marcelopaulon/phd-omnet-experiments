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

network DADCA_ACK_n100_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=193.87,105.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=237.61,55.65");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=297.52,61.92");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=285.43,188.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=345.51,221.54");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=424.45,176.39");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=468.65,87.41");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=382.67,129.68");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=301.24,127.93");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=536.39,120.14");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=571.70,205.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=593.46,281.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=504.27,179.23");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=209.67,184.03");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=133.97,174.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=53.92,158.03");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=36.22,245.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=70.64,302.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=6.96,344.52");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=13.51,436.59");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=70.72,469.85");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=159.04,469.71");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=105.94,377.84");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=131.38,281.31");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=13.11,61.53");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=82.21,21.41");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=155.99,14.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=156.09,339.88");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=206.34,400.19");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=152.59,554.63");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=227.18,577.55");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=236.94,484.90");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=249.90,271.59");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=236.07,332.38");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=371.20,390.24");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=430.04,449.42");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=500.77,505.26");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=492.59,437.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=593.87,483.27");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=679.76,523.90");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=615.66,381.73");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=674.38,236.57");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=698.77,161.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=656.73,94.40");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=716.67,10.76");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=761.84,163.87");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=818.28,188.35");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=883.72,132.17");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=818.01,98.76");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=895.67,274.81");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=954.17,207.48");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1042.51,220.54");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=997.14,321.11");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=953.52,364.24");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=844.09,325.13");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=921.69,427.72");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=859.30,445.86");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=795.52,411.37");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1143.39,236.86");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1094.27,275.78");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1131.09,169.95");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1249.56,240.21");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1114.15,99.45");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1155.54,8.62");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=994.19,36.86");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=915.07,77.95");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=974.81,115.84");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1045.82,128.22");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1205.70,175.20");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1056.27,50.69");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=735.22,84.42");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=646.00,6.95");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=602.30,61.96");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=851.12,41.71");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=788.28,272.40");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=854.99,542.62");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=758.70,562.94");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=800.41,514.75");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=742.89,334.03");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=687.88,298.32");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=905.74,487.39");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=722.92,423.63");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=863.81,609.46");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=945.82,647.45");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1043.37,633.59");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1035.76,705.02");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=997.61,790.73");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1055.86,763.52");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=1045.12,830.82");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=1008.88,884.04");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=1020.42,979.66");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=1085.25,1005.15");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1155.19,940.44");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=1145.46,864.03");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=1126.75,803.77");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1152.66,717.34");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=1217.35,749.58");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=1216.09,687.21");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=1304.00,812.29");
    }

}

