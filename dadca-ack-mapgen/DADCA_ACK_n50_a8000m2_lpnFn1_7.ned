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

network DADCA_ACK_n50_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=109.63,16.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=167.63,95.90");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=4.45,50.81");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=16.55,111.32");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=62.52,195.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=131.48,256.62");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=203.76,223.83");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=61.92,289.14");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=259.64,126.14");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=211.70,40.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=149.69,170.34");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=298.87,195.14");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=360.36,70.17");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=418.08,278.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=397.38,221.34");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=323.74,368.19");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=354.01,288.29");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=271.00,296.08");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=207.68,344.69");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=209.28,408.10");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=121.88,363.26");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=134.42,437.88");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=36.17,417.28");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=6.92,511.75");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=81.17,573.10");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=114.36,513.87");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=481.06,362.65");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=403.68,401.52");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=484.56,256.60");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=202.04,284.61");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=46.50,348.60");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=185.22,547.45");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=174.70,644.35");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=235.97,580.26");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=261.60,666.85");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=298.59,577.26");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=375.52,600.61");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=403.52,653.85");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=467.70,592.70");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=468.96,664.61");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=444.07,753.66");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=542.71,758.46");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=583.75,600.25");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=287.73,724.05");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=316.46,787.25");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=321.82,865.68");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=325.59,645.33");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=292.36,29.90");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=6.48,231.54");
    }

}

