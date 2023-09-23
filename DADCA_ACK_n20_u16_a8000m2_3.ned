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

network DADCA_ACK_n20_u16_a8000m2_3
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
        @display("i=misc/sensor;p=1108.34,7785.01");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2048.94,7755.22");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2633.75,7937.39");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3294.70,7987.37");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4067.17,7920.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4735.32,7778.06");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5193.24,7782.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=742.79,7710.62");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1729.59,7546.03");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5730.88,7610.06");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=350.02,7371.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1356.30,7374.43");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2219.65,7385.19");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2701.99,7282.94");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3031.50,7474.37");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3391.61,7371.43");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3776.09,7364.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4283.66,7498.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5010.20,7275.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=463.67,7020.21");
    }

}

