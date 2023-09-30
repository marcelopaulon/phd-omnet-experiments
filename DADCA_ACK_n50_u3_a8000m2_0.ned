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

network DADCA_ACK_n50_u3_a8000m2_0
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
        @display("i=misc/sensor;p=952.09,7892.27");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1482.59,7864.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2345.18,7882.24");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3308.89,7963.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3721.95,7813.61");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4373.07,7937.00");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5957.51,7905.32");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=653.52,7620.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1166.15,7594.92");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2942.26,7696.18");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4110.45,7678.64");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4681.52,7665.75");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5068.23,7731.63");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=102.57,7479.86");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1847.91,7454.51");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2403.63,7475.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3416.37,7464.86");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4337.07,7306.48");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5106.07,7307.73");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5523.34,7484.98");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=668.51,7255.80");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1116.54,7099.72");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2768.89,7122.67");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3563.25,7078.51");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4069.82,7030.18");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1564.08,6787.33");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2040.36,6832.23");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2441.45,6952.35");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4566.29,6924.14");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5041.45,6903.53");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5701.00,6883.49");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=211.66,6543.43");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=674.18,6659.47");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2670.75,6545.46");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3345.18,6714.37");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4189.05,6696.18");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4554.37,6561.02");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=501.11,6346.25");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1267.77,6430.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1716.26,6288.05");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2290.73,6426.01");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3917.47,6463.77");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5154.64,6429.96");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5843.43,6482.38");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=202.58,6138.99");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=955.28,6211.47");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1385.04,6088.28");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2902.21,6207.55");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3513.33,6204.09");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4413.77,6171.74");
    }

}

