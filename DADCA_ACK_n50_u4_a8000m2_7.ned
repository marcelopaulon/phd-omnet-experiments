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

network DADCA_ACK_n50_u4_a8000m2_7
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
        @display("i=misc/sensor;p=579.90,7963.23");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1146.81,7876.00");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1760.63,7885.87");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4175.76,7998.53");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4580.25,7861.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=842.05,7565.87");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2077.24,7701.43");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4151.87,7514.35");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5466.14,7506.00");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5859.38,7659.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=248.16,7464.96");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=594.60,7281.09");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1183.32,7391.54");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1520.53,7492.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2321.04,7447.28");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4509.55,7464.66");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5085.18,7469.73");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2012.20,7084.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2584.42,7190.04");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3609.22,7173.02");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4899.44,7010.98");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5946.10,7231.71");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=333.50,6861.09");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=828.39,6814.39");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1326.14,6837.79");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2770.41,6830.66");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3201.49,6808.44");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4141.67,6926.69");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4506.89,6852.09");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5524.80,6989.19");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1671.90,6561.82");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2099.49,6613.70");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3843.84,6648.14");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=795.97,6439.63");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2878.13,6310.33");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4137.77,6379.99");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4731.01,6509.10");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5098.47,6472.19");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5633.37,6466.13");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1326.16,6181.16");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2051.38,6076.52");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2500.90,6097.10");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3477.70,6253.30");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5324.59,6192.54");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=545.89,5949.45");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1074.10,5908.21");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1448.76,5837.81");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3127.52,6016.03");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3882.43,5965.94");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4258.60,5855.78");
    }

}

