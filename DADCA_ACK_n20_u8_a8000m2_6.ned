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

network DADCA_ACK_n20_u8_a8000m2_6
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
        @display("i=misc/sensor;p=1134.57,7859.97");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1812.07,7753.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4066.53,7938.81");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4516.30,7810.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=411.72,7534.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3354.11,7540.51");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5378.07,7673.28");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1092.01,7397.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1761.99,7346.01");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2256.10,7362.65");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2752.95,7470.35");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4067.69,7369.56");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5017.56,7339.22");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=931.44,7071.86");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4460.03,7222.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5371.74,7153.29");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5757.20,7124.50");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=174.35,6829.02");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1620.09,6976.67");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2235.09,6943.05");
    }

}

