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

network DADCA_ACK_n20_u2_a8000m2_7
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
        @display("i=misc/sensor;p=249.18,7857.74");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1203.67,7962.57");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2122.96,7985.56");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2489.35,7970.44");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4914.50,7781.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=855.69,7719.46");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1338.85,7584.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1762.47,7655.82");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3347.54,7564.92");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3677.81,7736.63");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4281.12,7697.16");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5242.74,7537.93");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=236.70,7322.81");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2626.59,7499.42");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3912.93,7384.40");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4582.79,7294.11");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=585.82,7171.46");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1152.96,7076.85");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1676.80,7025.12");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2057.59,7099.05");
    }

}

