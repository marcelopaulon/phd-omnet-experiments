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

network DADCA_ACK_n20_u3_a8000m2_4
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
        @display("i=misc/sensor;p=195.32,7970.61");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=649.96,7946.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1150.89,7923.20");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2460.28,7755.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2942.34,7802.13");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3327.10,7843.47");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3797.00,7826.67");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4629.08,7937.92");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5624.20,7854.68");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4351.38,7697.45");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5510.61,7519.69");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=523.89,7281.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=842.21,7484.23");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1200.61,7420.30");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1707.83,7468.50");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2350.91,7267.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3330.87,7385.52");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3754.60,7368.09");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4137.55,7373.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4518.22,7357.76");
    }

}

