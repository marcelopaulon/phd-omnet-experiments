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

network DADCA_ACK_n20_u3_a8000m2_9
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
        @display("i=misc/sensor;p=135.15,7987.82");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1198.39,7778.66");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2783.56,7981.27");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4420.73,7790.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5379.16,7833.19");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=691.66,7681.44");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1802.81,7749.12");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2349.29,7635.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4010.75,7560.82");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4924.15,7637.29");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5915.74,7744.46");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=375.90,7282.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1402.68,7356.22");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2800.06,7477.00");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3199.39,7419.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4308.84,7345.24");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=776.04,7236.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1116.45,7048.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1724.74,7196.04");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2255.61,7229.02");
    }

}

