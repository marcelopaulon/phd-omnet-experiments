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

network DADCA_ACK_n20_u8_a8000m2_9
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
        @display("i=misc/sensor;p=570.39,7895.73");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1023.12,7993.20");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2494.93,7950.23");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3903.32,7818.92");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1530.60,7653.28");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1931.65,7645.52");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3019.83,7633.89");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3486.26,7589.59");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4410.31,7690.81");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=191.49,7457.93");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=869.98,7485.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1277.20,7369.82");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2308.70,7475.35");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4973.11,7469.02");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=511.35,7201.24");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1782.45,7144.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2328.37,7111.41");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3786.83,7176.60");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4136.88,7223.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4767.69,7031.71");
    }

}

