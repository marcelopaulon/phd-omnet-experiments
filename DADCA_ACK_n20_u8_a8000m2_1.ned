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

network DADCA_ACK_n20_u8_a8000m2_1
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
        @display("i=misc/sensor;p=865.90,7843.82");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1860.51,7815.53");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2511.53,7809.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=508.76,7672.10");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2213.71,7552.15");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3078.78,7691.71");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4868.71,7739.13");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5315.17,7509.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5691.24,7665.12");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1115.23,7344.00");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1558.45,7475.20");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3769.67,7442.49");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4459.36,7431.21");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5761.40,7318.21");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1.21,7037.40");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=704.52,7092.84");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1885.21,7226.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2709.51,7216.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3192.75,7136.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4259.87,7046.71");
    }

}

