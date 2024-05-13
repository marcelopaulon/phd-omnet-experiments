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

network DADCA_ACK_n100_u16_a8000m2_0
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
        @display("i=misc/sensor;p=1564.10,7988.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3689.95,7804.54");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4164.57,7914.88");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4674.57,7877.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=507.03,7726.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1187.60,7634.85");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1624.01,7574.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2888.08,7633.04");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3257.59,7710.47");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4569.67,7534.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5129.51,7599.87");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3287.12,7312.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4146.85,7339.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5722.46,7496.09");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=917.30,7192.33");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1919.49,7048.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2363.18,7222.65");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2896.65,7236.56");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5870.16,7128.63");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=346.97,6957.40");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1467.77,6902.79");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3855.45,6878.65");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4844.56,6960.82");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5487.28,6915.67");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=107.92,6685.28");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=518.14,6604.41");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1055.03,6702.50");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2595.33,6678.87");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2989.46,6629.36");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3403.07,6645.95");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4262.79,6756.62");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4702.63,6609.98");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5322.05,6591.38");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5910.41,6650.73");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=216.39,6297.81");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1504.54,6309.33");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2079.45,6448.99");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3713.22,6426.69");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5636.67,6267.79");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=946.65,6091.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2464.95,6220.13");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3023.18,6067.33");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3502.59,6020.23");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3950.07,6151.61");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4317.58,6219.86");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5074.15,6175.63");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=310.51,5823.33");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2053.92,5871.26");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2689.99,5776.12");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4333.05,5849.92");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=4751.94,5923.00");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5505.07,5888.60");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5995.17,5814.49");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=845.56,5547.49");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1501.52,5635.45");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3930.74,5583.89");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5210.85,5650.80");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5653.47,5552.61");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2087.11,5481.04");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2476.95,5351.94");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3039.78,5289.38");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3465.64,5511.48");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4525.71,5493.54");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4973.00,5324.74");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=280.66,5211.09");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1438.82,5192.22");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2143.27,5074.09");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=3488.19,5042.50");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3897.30,5078.26");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=590.39,4861.40");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1188.29,4909.91");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3096.43,4820.25");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4431.97,4970.60");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5469.64,4979.74");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5823.33,4887.88");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=25.38,4608.20");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1538.81,4746.00");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=2644.22,4680.88");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4066.40,4649.95");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4880.63,4681.42");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=429.31,4479.25");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=924.94,4436.46");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2325.12,4469.86");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3468.49,4424.00");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4235.74,4295.37");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4603.84,4456.53");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5565.72,4416.00");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=648.73,4069.20");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1265.79,4250.96");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1951.31,4218.74");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2619.99,4046.54");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2891.15,4276.42");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3682.84,4140.04");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5913.14,4207.14");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=98.53,4015.55");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1014.33,4006.18");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1574.35,4006.37");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3076.35,3944.31");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3998.47,3969.96");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4403.83,3907.34");
    }

}

