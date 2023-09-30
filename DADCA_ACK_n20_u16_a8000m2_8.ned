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

network DADCA_ACK_n20_u16_a8000m2_8
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
        @display("i=misc/sensor;p=359.72,7759.67");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=996.15,7774.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1834.64,7795.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2525.50,7834.44");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2995.24,7830.84");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3460.56,7842.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4379.64,7766.92");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5117.17,7934.63");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2137.55,7614.51");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3728.27,7595.80");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5374.88,7694.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5712.86,7548.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=666.06,7321.00");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1261.81,7329.49");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1970.23,7263.10");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2418.30,7305.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2733.17,7480.74");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3143.91,7489.06");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4453.04,7384.64");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4923.03,7454.44");
    }

}

