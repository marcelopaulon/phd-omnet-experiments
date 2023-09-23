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

network DADCA_ACK_n100_u4_a8000m2_5
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
        @display("i=misc/sensor;p=847.25,7965.95");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1604.76,7796.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3558.17,7869.22");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3935.56,7797.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4270.61,7998.84");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4638.12,7896.91");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5132.11,7859.01");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5598.33,7924.40");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=993.88,7541.93");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2059.35,7596.49");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2974.17,7670.85");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=441.76,7423.09");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1417.50,7485.25");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1712.08,7276.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2396.35,7442.70");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3327.46,7381.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3888.52,7449.26");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4577.32,7360.86");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5098.81,7440.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5521.52,7462.07");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5994.81,7451.10");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=55.38,7025.41");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=505.05,7044.58");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=845.50,7140.46");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3744.66,7044.83");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5034.81,7074.81");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1119.57,6916.50");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1794.20,6838.67");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2287.04,6941.81");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3115.49,6866.28");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4604.65,6956.77");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5378.79,6919.12");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5741.46,6863.22");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2668.53,6547.34");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4188.86,6673.01");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4732.63,6524.54");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5195.74,6580.29");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5909.19,6528.44");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=606.39,6500.75");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1341.06,6394.16");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1788.11,6442.58");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3692.97,6446.97");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4326.44,6297.69");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=68.69,6078.09");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=834.07,6233.09");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2396.83,6204.98");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2751.13,6160.83");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3120.98,6240.19");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4843.23,6041.29");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5344.88,6225.94");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=640.45,5779.86");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3614.80,5979.88");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3970.72,5993.13");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5487.91,5802.57");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5852.34,5990.71");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1163.78,5760.29");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1535.45,5727.88");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2605.78,5646.37");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3077.86,5707.55");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3428.54,5589.12");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=529.57,5422.44");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1078.79,5346.51");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1988.18,5397.02");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4015.46,5401.11");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4417.57,5511.12");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5034.24,5514.08");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5694.65,5477.09");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=50.33,5058.17");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=591.95,5068.42");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1591.10,5187.94");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2322.52,5077.27");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2679.59,5133.12");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3293.32,5093.49");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3638.88,5229.21");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=4611.04,5154.22");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=5427.09,5164.89");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1133.60,4940.47");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3811.22,4817.76");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4246.55,4859.70");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=7.57,4702.61");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=802.55,4764.57");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=1404.92,4678.06");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1966.14,4762.27");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2583.78,4773.02");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4066.77,4557.38");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4582.16,4744.82");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5075.80,4731.00");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=5773.71,4753.92");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=246.76,4401.42");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2269.01,4493.78");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3040.34,4470.53");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4415.89,4434.66");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4845.43,4450.84");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5268.35,4366.92");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=762.30,4145.16");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1103.34,4257.75");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1624.87,4171.68");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=2542.00,4231.62");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3420.28,4287.66");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3821.78,4146.70");
    }

}

