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

network DADCA_ACK_n100_a8000m2_6
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
        @display("i=misc/sensor;p=90.43,7980.68");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=271.16,7991.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=738.48,7977.39");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=836.71,7959.38");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=913.61,7971.86");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=972.97,7986.20");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=162.41,7934.21");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=286.50,7916.59");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=349.04,7933.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=455.44,7945.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=556.83,7945.73");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=621.64,7926.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=998.07,7918.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=72.57,7893.96");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=495.21,7897.36");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=568.34,7876.96");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=674.01,7879.74");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=738.37,7871.53");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=809.30,7875.93");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=23.82,7834.55");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=127.48,7851.84");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=256.19,7830.45");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=371.01,7865.48");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=447.93,7852.54");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=908.91,7863.35");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=170.72,7794.54");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=334.93,7807.15");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=530.74,7824.34");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=602.46,7792.02");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=665.63,7785.11");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=750.93,7792.33");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=850.63,7821.74");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=84.19,7756.32");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=508.29,7758.20");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=957.82,7783.00");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=262.75,7729.44");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=426.10,7738.86");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=539.02,7699.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=609.51,7726.42");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=710.64,7726.73");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=846.00,7732.89");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=974.99,7714.28");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=35.93,7696.03");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=168.85,7676.22");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=431.18,7659.91");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=762.02,7662.72");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=903.99,7687.60");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=72.54,7637.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=207.57,7617.59");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=272.16,7649.56");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=553.80,7615.97");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=651.75,7649.08");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=812.94,7622.74");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=964.85,7640.65");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=155.40,7576.68");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=367.26,7607.36");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=475.29,7610.16");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=713.85,7599.79");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=913.30,7606.97");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=22.02,7532.65");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=249.05,7555.72");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=313.88,7561.76");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=425.08,7541.47");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=503.99,7543.15");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=605.43,7530.41");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=764.62,7563.16");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=859.77,7548.88");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=957.51,7561.40");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=78.39,7496.69");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=330.78,7500.53");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=665.79,7509.88");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=787.27,7501.95");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=151.94,7478.10");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=372.49,7447.82");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=480.49,7446.29");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=583.53,7469.64");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=915.39,7446.86");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=11.46,7398.55");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=140.34,7414.67");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=242.22,7437.31");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=638.73,7414.98");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=754.75,7414.97");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=844.98,7414.84");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=991.61,7405.98");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=210.40,7374.48");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=329.67,7363.77");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=401.26,7377.81");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=499.07,7387.86");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=713.51,7369.68");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=820.64,7333.19");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=880.06,7350.73");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=49.15,7293.63");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=132.63,7287.15");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=305.77,7305.35");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=412.52,7300.32");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=475.97,7283.26");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=538.95,7268.13");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=637.58,7281.00");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=709.29,7306.95");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=954.28,7307.29");
    }

}

