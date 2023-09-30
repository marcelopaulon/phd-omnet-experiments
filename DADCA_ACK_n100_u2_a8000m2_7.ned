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

network DADCA_ACK_n100_u2_a8000m2_7
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
        @display("i=misc/sensor;p=14.72,7796.91");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=529.17,7775.00");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1145.87,7849.97");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2771.46,7807.84");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3283.82,7823.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5292.98,7758.80");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5676.68,7933.78");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=850.89,7537.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2239.89,7592.34");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3605.64,7613.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4624.51,7735.60");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5955.60,7709.40");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=177.98,7397.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2722.76,7387.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4118.81,7485.35");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5317.49,7367.34");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=590.04,7186.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1267.41,7144.09");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1816.28,7162.91");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2448.93,7062.77");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3221.60,7171.75");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4597.62,7231.94");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5724.20,7016.31");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=160.55,6786.30");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=626.63,6838.03");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=980.67,6777.71");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3728.61,7001.28");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4717.15,6793.61");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5364.79,6766.19");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1593.71,6704.94");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2153.34,6582.00");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2515.44,6645.24");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2924.03,6693.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3507.98,6653.79");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4190.36,6609.24");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5941.83,6704.03");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=228.99,6314.56");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=667.90,6476.78");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5152.57,6354.32");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5649.09,6344.67");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=649.11,6028.66");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1010.17,6119.71");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1674.39,6102.34");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3060.62,6225.60");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3684.40,6163.17");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4082.61,6023.56");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4689.35,6109.46");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=133.83,5879.71");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2276.96,5980.75");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2820.99,5819.45");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4439.52,5828.50");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5353.21,6005.37");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5765.17,5937.15");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1204.15,5665.49");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1978.82,5758.82");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3312.35,5658.38");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3802.15,5588.67");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5753.21,5560.67");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=320.73,5486.90");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=717.08,5349.72");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1656.77,5386.37");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2215.16,5406.06");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2634.04,5520.84");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3001.70,5391.12");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3507.15,5297.99");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4562.23,5323.34");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5060.27,5518.60");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5433.48,5363.32");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2655.09,5135.06");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5746.15,5131.90");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=124.25,4974.71");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=675.99,4908.30");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1083.29,4964.52");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1939.97,4982.17");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4031.37,4952.93");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5317.06,4941.65");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1374.44,4746.57");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2888.64,4747.85");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3333.88,4735.10");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=3776.76,4592.25");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4899.88,4750.11");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=49.39,4324.17");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=438.07,4501.55");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=983.70,4436.92");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1921.42,4447.75");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=2477.49,4397.57");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2987.13,4314.82");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4434.53,4294.49");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=5285.80,4484.91");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=362.04,4152.40");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=850.68,4107.18");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1203.11,4082.03");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3386.85,4114.87");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5001.25,4069.76");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=221.26,3823.05");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2065.76,3932.62");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2459.00,3979.78");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3858.54,3919.05");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5388.98,3909.41");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=780.13,3629.27");
    }

}

