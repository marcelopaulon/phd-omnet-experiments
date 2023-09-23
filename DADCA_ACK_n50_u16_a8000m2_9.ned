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

network DADCA_ACK_n50_u16_a8000m2_9
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
        @display("i=misc/sensor;p=252.74,7769.32");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1022.40,7788.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1454.94,7925.92");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1927.89,7920.49");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4147.58,7754.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4513.73,7827.32");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=620.39,7740.68");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1444.79,7565.98");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4911.75,7739.34");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5386.49,7563.02");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=36.93,7371.12");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=706.55,7380.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1873.00,7322.07");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2318.51,7358.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3164.96,7305.47");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1238.83,7235.06");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2685.19,7112.35");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3608.04,7213.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4300.39,7109.74");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4835.92,7150.66");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5371.72,7174.73");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=318.66,6978.90");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=722.43,6827.24");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1083.91,6854.89");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1678.48,6984.93");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2862.17,6765.86");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3380.09,6850.00");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4037.95,6793.05");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5646.92,6863.88");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1404.09,6566.94");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1983.81,6651.79");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2503.25,6606.57");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4716.97,6524.77");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5168.39,6549.44");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=297.84,6468.81");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2016.02,6300.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3661.18,6434.97");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5962.53,6439.08");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1487.99,6175.97");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2773.14,6076.48");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3316.38,6213.08");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4181.80,6163.25");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4616.57,6109.90");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5068.87,6084.36");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5652.19,6245.42");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=293.24,5897.46");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1859.83,5838.82");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2273.43,5939.25");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3316.24,5852.02");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3864.79,5999.21");
    }

}

