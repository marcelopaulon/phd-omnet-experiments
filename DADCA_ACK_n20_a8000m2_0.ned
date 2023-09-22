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

network DADCA_ACK_n20_a8000m2_0
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
        @display("i=misc/sensor;p=396.79,7899.81");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=911.37,7976.23");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2059.49,7969.41");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3618.77,7798.33");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5382.91,7987.68");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=771.54,7616.62");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1498.84,7734.38");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2127.42,7623.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4217.24,7715.50");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4989.56,7603.88");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=30.37,7317.11");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=494.80,7293.38");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1114.35,7472.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2759.52,7471.60");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3309.01,7434.03");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3907.89,7286.62");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4478.36,7310.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5318.99,7482.83");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1064.93,7059.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1625.75,7242.80");
    }

}

