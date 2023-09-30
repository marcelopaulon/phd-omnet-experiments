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

network DADCA_ACK_n20_u2_a8000m2_1
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
        @display("i=misc/sensor;p=66.72,7906.69");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2020.01,7992.96");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3407.63,7934.90");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3829.35,7789.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4185.17,7797.20");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5257.86,7922.78");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=714.61,7717.83");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1168.98,7619.36");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1849.30,7571.00");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2323.26,7591.68");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4605.82,7676.11");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5629.00,7516.48");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=305.15,7385.96");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1612.60,7273.21");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2684.78,7329.43");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3675.12,7269.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4137.06,7345.23");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5111.23,7330.44");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5985.38,7485.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2259.33,7102.24");
    }

}

