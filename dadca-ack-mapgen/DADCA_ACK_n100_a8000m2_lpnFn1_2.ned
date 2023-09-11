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
        @display("i=misc/sensor,p=167.05,110.88");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=186.49,17.55");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=250.64,51.28");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=206.30,197.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=52.53,37.55");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=103.80,180.85");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=242.43,142.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=294.39,208.06");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=308.12,271.84");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=236.66,309.85");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=160.10,315.43");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=149.59,379.08");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=205.70,411.17");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=163.55,476.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=228.46,532.59");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=264.68,609.24");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=287.68,480.67");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=325.46,534.30");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=383.14,402.99");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=472.22,430.03");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=521.40,352.24");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=494.21,281.75");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=489.02,216.13");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=577.37,239.10");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=571.02,172.57");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=513.30,90.99");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=456.68,47.18");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=432.48,105.92");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=344.41,35.64");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=346.68,161.45");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=371.32,318.88");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=431.97,288.34");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=440.46,351.35");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=299.90,99.04");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=372.12,222.19");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=482.25,147.12");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=424.81,175.09");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=638.66,204.43");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=683.13,249.99");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=760.27,295.78");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=836.24,295.72");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=689.55,332.88");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=714.16,416.08");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=632.54,304.52");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=317.86,366.07");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=367.59,477.47");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=463.05,561.84");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=542.57,506.56");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=562.11,567.79");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=462.79,690.72");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=398.69,718.34");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=335.91,729.41");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=375.50,785.42");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=323.21,651.64");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=380.85,625.82");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=493.39,634.23");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=581.84,630.43");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=603.58,403.55");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=629.53,469.31");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=673.23,541.99");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=748.57,530.76");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=734.83,606.65");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=422.36,503.08");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=111.59,513.32");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=63.32,426.58");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=34.71,361.50");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=25.61,300.87");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=25.17,211.27");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1.22,146.56");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=93.69,311.40");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=144.37,254.00");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=50.81,514.07");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=17.82,572.49");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=180.35,599.71");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=122.42,575.32");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=639.97,117.06");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=623.19,50.04");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=682.01,17.90");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=513.83,7.33");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=323.30,430.18");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=156.50,666.73");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=71.35,692.47");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=32.75,637.90");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=166.24,767.04");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=113.44,9.08");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=83.32,238.33");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=252.43,704.63");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=255.20,768.26");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=255.83,856.02");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=251.40,927.50");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=314.58,895.95");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=381.13,877.61");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=284.24,980.53");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=357.04,1010.74");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=452.79,1092.56");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=433.62,1028.20");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=503.02,1138.10");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=599.55,1118.35");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=570.29,1048.22");
    }

}

