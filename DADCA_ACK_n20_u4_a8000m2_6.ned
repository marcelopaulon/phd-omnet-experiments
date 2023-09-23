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

network DADCA_ACK_n20_u4_a8000m2_6
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
        @display("i=misc/sensor;p=30.08,7852.51");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=516.20,7855.44");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1524.20,7975.50");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3507.17,7785.66");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5970.71,7914.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=991.27,7649.11");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2298.36,7642.35");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4203.33,7742.32");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4845.91,7544.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5204.45,7622.20");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=235.15,7272.50");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1385.20,7439.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1812.45,7343.16");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2605.05,7354.51");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2950.20,7461.69");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3784.56,7484.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5826.09,7366.80");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=672.40,7159.50");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3398.40,7224.14");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3786.86,7132.04");
    }

}

