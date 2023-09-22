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

network DADCA_ACK_n100_a8000m2_1
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
        @display("i=misc/sensor;p=101.96,7981.84");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=200.95,7998.34");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=285.73,7970.94");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=401.36,7992.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=714.36,7999.72");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=775.84,7969.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=855.25,7988.98");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=973.05,7986.70");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=198.38,7927.73");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=335.90,7921.76");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=441.75,7930.11");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=504.95,7953.66");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=589.42,7934.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=667.20,7951.52");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=921.37,7951.20");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=18.83,7907.93");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=113.82,7872.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=233.61,7876.51");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=500.08,7888.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=649.91,7889.62");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=710.99,7895.97");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=823.77,7885.30");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=967.06,7884.95");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=29.39,7846.79");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=192.00,7831.13");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=288.70,7839.60");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=481.31,7827.54");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=902.94,7839.79");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=119.84,7798.28");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=407.83,7825.14");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=567.99,7799.24");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=744.24,7811.19");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=10.39,7769.48");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=182.40,7764.65");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=311.92,7782.28");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=489.11,7753.08");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=631.53,7752.80");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=692.55,7758.58");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=840.74,7783.33");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=924.70,7778.20");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=69.88,7721.69");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=187.45,7698.86");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=256.86,7715.10");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=406.87,7733.00");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=772.77,7707.20");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=364.89,7674.08");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=456.72,7672.13");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=604.45,7697.34");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=687.62,7686.62");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=976.35,7680.02");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=0.01,7651.36");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=142.19,7650.79");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=641.57,7625.14");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=814.75,7649.19");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=913.95,7641.56");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=145.75,7587.89");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=247.69,7608.94");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=349.58,7581.25");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=542.43,7604.23");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=743.91,7581.92");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=814.66,7580.98");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=981.41,7578.97");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=48.11,7567.32");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=217.20,7539.27");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=388.94,7532.58");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=460.52,7544.98");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=628.82,7545.64");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=694.18,7528.68");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=114.49,7496.77");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=279.48,7504.98");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=456.37,7482.52");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=779.47,7522.17");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=887.94,7523.78");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1.22,7472.13");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=144.32,7442.58");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=366.54,7455.21");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=585.95,7465.03");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=667.92,7468.58");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=734.87,7449.24");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=927.77,7445.41");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=87.58,7406.18");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=198.07,7412.26");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=291.50,7412.40");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=499.60,7435.45");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=847.06,7419.50");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=404.80,7394.85");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=613.86,7357.05");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=951.01,7370.33");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=53.20,7348.56");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=128.69,7341.03");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=235.45,7310.37");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=374.56,7340.54");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=490.86,7322.53");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=707.44,7310.17");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=791.83,7338.93");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=890.84,7352.62");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=317.59,7284.32");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=425.95,7280.52");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=939.68,7291.60");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=18.85,7235.11");
    }

}

