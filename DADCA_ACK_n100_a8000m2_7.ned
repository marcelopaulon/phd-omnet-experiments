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

network DADCA_ACK_n100_a8000m2_7
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
        @display("i=misc/sensor;p=4835.29,7860.04");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=5465.26,7974.61");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=768.25,7608.81");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1741.23,7601.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2612.85,7602.56");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3450.71,7610.87");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4089.50,7542.96");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4600.78,7532.24");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=428.57,7328.20");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2956.03,7387.74");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4869.38,7246.15");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5232.34,7391.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5921.04,7465.10");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=840.76,7147.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1722.80,7146.85");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2248.73,7069.17");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3309.69,7165.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3721.66,7025.68");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4107.66,7025.67");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4508.87,6987.29");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5345.77,6983.43");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5858.32,7042.23");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=834.41,6722.56");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1199.99,6931.80");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2951.32,6927.85");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4899.78,6770.58");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=423.39,6616.00");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1397.48,6513.67");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1892.94,6479.66");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2645.35,6625.51");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3109.37,6527.77");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3484.01,6683.17");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4037.43,6637.86");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=837.15,6316.20");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2376.62,6332.96");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3536.53,6314.19");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4184.98,6301.73");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4622.21,6287.39");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5571.39,6391.04");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=416.00,6073.53");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1286.84,5991.71");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1888.68,5968.53");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2500.71,5955.44");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2923.84,5995.77");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3366.02,5943.13");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4999.16,6047.63");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5487.09,5975.20");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5848.04,6051.34");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4099.44,5784.77");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=406.43,5588.63");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=883.26,5537.09");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1402.00,5453.85");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2389.67,5501.62");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4215.09,5434.31");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=4635.01,5597.46");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5086.05,5480.92");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5683.63,5603.90");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2029.86,5294.68");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3036.37,5379.74");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3606.41,5338.50");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4570.91,5155.35");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5990.87,5339.89");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=298.27,5109.61");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=823.70,5084.20");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2595.15,5026.56");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3266.48,4911.65");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4240.36,4908.94");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5692.28,5118.82");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=785.12,4675.27");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1244.29,4659.54");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2286.44,4705.45");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3579.33,4691.30");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5061.10,4888.08");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5588.33,4692.35");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1917.20,4601.58");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4517.03,4605.19");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=399.15,4363.50");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1217.06,4201.76");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1918.49,4223.27");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2536.67,4261.76");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3101.99,4218.91");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=3492.15,4181.56");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=4041.38,4376.18");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5207.46,4308.33");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5839.29,4213.04");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1592.76,4027.82");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2342.87,3909.01");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4116.20,3880.46");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4903.02,4096.31");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=123.62,3839.71");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=675.04,3727.89");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1158.76,3706.53");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1818.61,3619.05");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2865.93,3659.84");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5439.75,3708.51");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=236.33,3409.56");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2236.25,3536.22");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3127.85,3361.55");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3576.86,3540.86");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4282.87,3532.21");
    }

}

