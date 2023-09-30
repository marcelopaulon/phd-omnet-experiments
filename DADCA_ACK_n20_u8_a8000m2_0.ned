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

network DADCA_ACK_n20_u8_a8000m2_0
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
        @display("i=misc/sensor;p=341.55,7980.10");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=939.93,7757.16");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1381.12,7963.55");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1840.53,7843.17");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2319.64,7914.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2786.93,7964.64");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3891.23,7958.67");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4397.04,7857.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4748.72,7889.82");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5193.94,7859.73");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5598.83,7973.60");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2980.84,7646.86");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3491.98,7679.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5760.21,7620.65");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=291.84,7311.57");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1268.09,7368.67");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2255.42,7356.18");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2614.98,7437.41");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4085.24,7497.31");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4581.55,7289.01");
    }

}

