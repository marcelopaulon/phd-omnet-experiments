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

network DADCA_ACK_n20_u8_a8000m2_2
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
        @display("i=misc/sensor;p=47.10,7887.11");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=572.13,7852.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1790.92,7904.89");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2140.01,7859.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3236.22,7980.08");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4706.28,7898.46");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5193.88,7865.00");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5720.96,7942.73");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1105.74,7737.66");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1472.33,7658.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2812.00,7621.64");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3931.33,7731.94");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4989.27,7512.55");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5494.57,7674.85");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=603.92,7271.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1873.89,7345.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2408.45,7420.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3010.91,7293.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3352.31,7472.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4338.61,7377.04");
    }

}

