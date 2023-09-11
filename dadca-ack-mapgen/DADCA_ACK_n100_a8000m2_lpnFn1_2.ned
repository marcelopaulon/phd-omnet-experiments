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

network DADCA_ACK_n100_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=159.76,32.87");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=225.35,56.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=162.74,105.29");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=136.93,163.14");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=116.95,226.26");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=194.49,180.18");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=119.00,289.90");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=62.37,336.86");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=206.97,304.06");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=290.28,297.69");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=355.83,339.23");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=355.80,265.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=416.75,248.60");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=477.45,280.25");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=362.76,160.26");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=445.36,83.35");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=356.54,94.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=353.46,13.96");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=294.14,90.22");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=215.98,242.47");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=265.83,179.95");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=57.57,161.31");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=39.65,224.91");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=144.13,369.40");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=509.68,174.59");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=578.73,169.92");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=569.84,238.62");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=606.33,290.11");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=554.00,331.44");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=662.49,254.39");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=674.03,322.73");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=592.54,388.17");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=661.55,131.65");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=529.04,106.77");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=510.75,14.77");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=598.14,86.11");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=683.04,55.79");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=740.26,85.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=831.31,69.67");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=806.29,135.37");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=866.06,161.87");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=928.71,188.01");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=916.20,255.97");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=841.59,260.56");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=946.63,361.41");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=859.53,401.01");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=784.21,445.47");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=732.38,406.90");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=731.92,276.80");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=794.21,323.77");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=784.27,219.59");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=746.12,156.20");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=885.95,464.16");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=694.94,540.91");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=690.64,468.88");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=512.47,485.22");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=543.22,557.91");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=516.24,619.43");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=460.80,576.60");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=499.29,717.19");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=455.86,439.58");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=436.25,507.28");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=402.24,710.22");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=422.70,627.29");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=543.44,786.61");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=600.47,809.93");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=531.73,879.69");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=578.60,920.32");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=452.28,926.88");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=453.04,1026.31");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=414.53,1082.23");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=486.00,1110.40");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=459.42,1184.72");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=407.87,1232.83");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=418.52,1298.70");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=516.27,1321.73");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=581.51,1301.31");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=626.13,1254.29");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=642.49,1193.59");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=712.74,1137.31");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=635.65,1088.12");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=635.68,1014.37");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=542.97,1001.54");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=320.62,797.12");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=261.78,906.04");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=334.83,928.19");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=353.82,1010.38");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=290.80,981.36");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=181.41,972.37");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=101.53,927.89");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=39.94,925.26");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=0.17,984.36");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=57.36,1010.54");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=151.67,837.52");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=75.68,843.16");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=1.50,855.67");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=58.95,774.80");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=84.28,681.12");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=147.94,728.86");
    }

}

