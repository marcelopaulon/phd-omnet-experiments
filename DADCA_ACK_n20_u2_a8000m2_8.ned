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

network DADCA_ACK_n20_u2_a8000m2_8
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
        @display("i=misc/sensor;p=441.54,7808.81");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=869.12,7951.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1594.44,7997.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3482.26,7762.44");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4280.43,7884.84");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5907.34,7774.38");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1449.99,7668.99");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2021.27,7739.34");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2359.71,7591.47");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2992.14,7747.76");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3845.00,7521.53");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4997.31,7645.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5422.08,7678.14");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=642.04,7477.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1030.21,7358.18");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3200.64,7289.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4463.99,7457.77");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=137.46,7154.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1712.19,7172.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2292.73,7240.90");
    }

}

